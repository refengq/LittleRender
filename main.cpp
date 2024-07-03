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
	mat<4, 4, float> uniform_M;   //  视图变换矩阵
	mat<4, 4, float> uniform_MIT; //  视图变换矩阵逆转置
	mat<4, 4, float> uniform_Mshadow; // 
	mat<2, 3, float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
	mat<3, 3, float> varying_tri; // triangle coordinates before Viewport transform, written by VS, read by FS

	Shader(Matrix M, Matrix MIT, Matrix MS) : uniform_M(M), uniform_MIT(MIT), uniform_Mshadow(MS), varying_uv(), varying_tri() {}

	virtual Vec4f vertex(int iface, int nthvert)//获取对应面对应点的uv坐标,和屏幕坐标,归一化的屏幕坐标
	{
		//setcol 设置对应列的函数,这里是把nthvert列第一行设置为u,第二行设置为v 
		varying_uv.set_col(nthvert, model->uv(iface, nthvert));
		Vec4f gl_Vertex = Viewport * Projection * ModelView * embed<4>(model->vert(iface, nthvert));// 变换到屏幕空间,不过此时w不为1
		varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));//进行归一化将w变为1的坐标
		return gl_Vertex;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)
	{
		Vec4f sb_p = uniform_Mshadow * embed<4>(varying_tri * bar); //使用重心插值后转换为阴影buffer的值 
		sb_p = sb_p / sb_p[3];                                     //归一化
		int idx = int(sb_p[0]) + int(sb_p[1]) * width; // 算出对应序列值
		float shadow = .3 + .7 * (shadowbuffer[idx] < sb_p[2]); // 阴影赋值为0.3 否则1
		Vec2f uv = varying_uv * bar;                 // uv插值 
		Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize(); // 用逆矩阵算出normal
		Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize(); // light vector
		Vec3f r = (n * (n * l * 2.f) - l).normalize();   // reflected light
		//这里主要是套公式,故不在描述
		float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
		float diff = std::max(0.f, n * l);
		TGAColor c = model->diffuse(uv);
		for (int i = 0; i < 3; i++) //进行RGB三种颜色的计算,考虑环境光,漫反射,直射的近似phong模型
			color[i] = std::min<float>(20 + c[i] * shadow * (1.2 * diff + .6 * spec), 255);
		return false;
	}
};

struct DepthShader : public IShader
{
	mat<3, 3, float> varying_tri;

	DepthShader() : varying_tri() {}

	virtual Vec4f vertex(int iface, int nthvert)//与上述Shader几乎一致不再说明
	{
		Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
		gl_Vertex = Viewport * Projection * ModelView * gl_Vertex;
		varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
		return gl_Vertex;
	}

	virtual bool fragment(Vec3f bar, TGAColor& color)//bar 重心坐标 进行插值赋予颜色 画出深度图
	{
		Vec3f p = varying_tri * bar;
		color = TGAColor(255, 255, 255) * (p.z / depth);
		return false;
	}
};
//编译后运行 ./name.exe obj/african_head.obj 即可运行
int main(int argc, char** argv)
{
	if (2 > argc)
	{
		std::cerr << "Usage: " << argv[0] << "obj/model.obj" << std::endl;
		return 1;
	}

	float* zbuffer = new float[width * height];//深度图
	shadowbuffer = new float[width * height];//阴影图
	for (int i = width * height; --i; )//赋初值
	{
		zbuffer[i] = shadowbuffer[i] = -std::numeric_limits<float>::max();
	}

	model = new Model(argv[1]);
	//light_dir.normalize();
	{
		TGAImage depth(width, height, TGAImage::RGB);
		lookat(light_dir, center, up);//第一遍将视角放在相机获得阴影坐标
		/*
		   x：视口的左下角 x 坐标，等于屏幕宽度的八分之一。
		   y：视口的左下角 y 坐标，等于屏幕高度的八分之一。
		   w：视口的宽度，等于屏幕宽度的四分之三（即，屏幕宽度减去两侧各八分之一的空白）。
		   h：视口的高度，等于屏幕高度的四分之三（即，屏幕高度减去上下各八分之一的空白）
		   所以结果是从预留的上下左右1/8开始显示中间的3/4的区域
		   x=3/8*width*x+width/2
		   y=3/8*height*x+height/2
		   z=1000.z+1000 viewport里depth=2000
		*/
		viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
		//projection(-1.f / (light_dir - center).norm());//使用透视投影替换正交投影
		projection(0);//只计算阴影故正交变换即可

		DepthShader depthshader;
		Vec4f screen_coords[3];
		for (int i = 0; i < model->nfaces(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				screen_coords[j] = depthshader.vertex(i, j);//返回的是没有归一化的坐标
			}
			triangle(screen_coords, depthshader, depth, shadowbuffer);//进行染色
		}
		depth.flip_vertically();
		depth.write_tga_file("depth.tga");
	}

	Matrix M = Viewport * Projection * ModelView;//保存第一部的函数,用来将之后的坐标映射到阴影图的坐标里

	{
		TGAImage frame(width, height, TGAImage::RGB);
		//视角变换
		lookat(eye, center, up);
		viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
		projection(-1.f / (eye - center).norm());
		//初始化 视角变换矩阵 逆转值矩阵 
		// M * (Viewport * Projection * ModelView).invert()  第一步通过逆变化变回原来的坐标,
		// 再通过阴影矩阵的变化找到阴影坐标进行判断
		Shader shader(ModelView, (Projection * ModelView).invert_transpose(), M * (Viewport * Projection * ModelView).invert());
		Vec4f screen_coords[3];//循环与depthrender一致
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

