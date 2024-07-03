#include <vector>
#include <limits>
#include <iostream>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model* model = NULL;
float* shadowbuffer = NULL;

const int width = 800;
const int height = 800;

Vec3f light_dir(1, 1, 0);
Vec3f       eye(3, 1, 6);
Vec3f    center(0, 0, 0);
Vec3f        up(0, 1, 0);

struct Shader : public IShader
{
	mat<4, 4, float> uniform_M;   //  ��ͼ�任����
	mat<4, 4, float> uniform_MIT; //  ��ͼ�任������ת��
	mat<4, 4, float> uniform_Mshadow; // 
	mat<2, 3, float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
	mat<3, 3, float> varying_tri; // triangle coordinates before Viewport transform, written by VS, read by FS

	Shader(Matrix M, Matrix MIT, Matrix MS) : uniform_M(M), uniform_MIT(MIT), uniform_Mshadow(MS), varying_uv(), varying_tri() {}

	virtual Vec4f vertex(int iface, int nthvert)//��ȡ��Ӧ���Ӧ���uv����,����Ļ����,��һ������Ļ����
	{
		//setcol ���ö�Ӧ�еĺ���,�����ǰ�nthvert�е�һ������Ϊu,�ڶ�������Ϊv 
		varying_uv.set_col(nthvert, model->uv(iface, nthvert));
		Vec4f gl_Vertex = Viewport * Projection * ModelView * embed<4>(model->vert(iface, nthvert));// �任����Ļ�ռ�,������ʱw��Ϊ1
		varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));//���й�һ����w��Ϊ1������
		return gl_Vertex;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)
	{
		Vec4f sb_p = uniform_Mshadow * embed<4>(varying_tri * bar); //ʹ�����Ĳ�ֵ��ת��Ϊ��Ӱbuffer��ֵ 
		sb_p = sb_p / sb_p[3];                                     //��һ��
		int idx = int(sb_p[0]) + int(sb_p[1]) * width; // �����Ӧ����ֵ
		float shadow = .3 + .7 * (shadowbuffer[idx] < sb_p[2]); // ��Ӱ��ֵΪ0.3 ����1
		Vec2f uv = varying_uv * bar;                 // uv��ֵ 
		Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize(); // ����������normal
		Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize(); // light vector
		Vec3f r = (n * (n * l * 2.f) - l).normalize();   // reflected light
		//������Ҫ���׹�ʽ,�ʲ�������
		float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
		float diff = std::max(0.f, n * l);
		TGAColor c = model->diffuse(uv);
		for (int i = 0; i < 3; i++) //����RGB������ɫ�ļ���,���ǻ�����,������,ֱ��Ľ���phongģ��
			color[i] = std::min<float>(20 + c[i] * shadow * (1.2 * diff + .6 * spec), 255);
		return false;
	}
};

struct DepthShader : public IShader
{
	mat<3, 3, float> varying_tri;

	DepthShader() : varying_tri() {}

	virtual Vec4f vertex(int iface, int nthvert)//������Shader����һ�²���˵��
	{
		Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
		gl_Vertex = Viewport * Projection * ModelView * gl_Vertex;
		varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
		return gl_Vertex;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)//bar �������� ���в�ֵ������ɫ �������ͼ
	{
		Vec3f p = varying_tri * bar;
		color = TGAColor(255, 255, 255) * (p.z / depth);
		return false;
	}
};

int main(int argc, char** argv)
{
	if (2 > argc)
	{
		std::cerr << "Usage: " << argv[0] << "obj/model.obj" << std::endl;
		return 1;
	}

	float* zbuffer = new float[width * height];//���ͼ
	shadowbuffer = new float[width * height];//��Ӱͼ
	for (int i = width * height; --i; )//����ֵ
	{
		zbuffer[i] = shadowbuffer[i] = -std::numeric_limits<float>::max();
	}

	model = new Model(argv[1]);
	//light_dir.normalize();
	{
		TGAImage depth(width, height, TGAImage::RGB);
		lookat(light_dir, center, up);//��һ�齫�ӽǷ�����������Ӱ����
		/*
		   x���ӿڵ����½� x ���꣬������Ļ��ȵİ˷�֮һ��
		   y���ӿڵ����½� y ���꣬������Ļ�߶ȵİ˷�֮һ��
		   w���ӿڵĿ�ȣ�������Ļ��ȵ��ķ�֮����������Ļ��ȼ�ȥ������˷�֮һ�Ŀհף���
		   h���ӿڵĸ߶ȣ�������Ļ�߶ȵ��ķ�֮����������Ļ�߶ȼ�ȥ���¸��˷�֮һ�Ŀհף�
		   ���Խ���Ǵ�Ԥ������������1/8��ʼ��ʾ�м��3/4������
		   x=3/8*width*x+width/2
		   y=3/8*height*x+height/2
		   z=1000.z+1000 viewport��depth=2000
		*/
		viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
		//projection(-1.f / (light_dir - center).norm());//ʹ��͸��ͶӰ�滻����ͶӰ
		projection(0);//ֻ������Ӱ�������任����

		DepthShader depthshader;
		Vec4f screen_coords[3];
		for (int i = 0; i < model->nfaces(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				screen_coords[j] = depthshader.vertex(i, j);//���ص���û�й�һ��������
			}
			triangle(screen_coords, depthshader, depth, shadowbuffer);//����Ⱦɫ
		}
		depth.flip_vertically();
		depth.write_tga_file("depth.tga");
	}

	Matrix M = Viewport * Projection * ModelView;//�����һ���ĺ���,������֮�������ӳ�䵽��Ӱͼ��������

	{
		TGAImage frame(width, height, TGAImage::RGB);
		//�ӽǱ任
		lookat(eye, center, up);
		viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
		projection(-1.f / (eye - center).norm());
		//��ʼ�� �ӽǱ任���� ��תֵ���� 
		// M * (Viewport * Projection * ModelView).invert()  ��һ��ͨ����仯���ԭ��������,
		// ��ͨ����Ӱ����ı仯�ҵ���Ӱ��������ж�
		Shader shader(ModelView, (Projection * ModelView).invert_transpose(), M * (Viewport * Projection * ModelView).invert());
		Vec4f screen_coords[3];//ѭ����depthrenderһ��
		for (int i = 0; i < model->nfaces(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				screen_coords[j] = shader.vertex(i, j);
			}
			triangle(screen_coords, shader, frame, zbuffer);
		}
		frame.flip_vertically();
		frame.write_tga_file("framebuffer.tga");
	}

	delete model;
	delete[] zbuffer;
	delete[] shadowbuffer;
	return 0;
}

