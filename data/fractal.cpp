#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float f32;
typedef double f64;

#pragma pack(push, 1)
struct bmp_file_header
{
    //file header
    char bfType[2];
    u32 bfSize;
    u16 bfReserved1;
    u16 bfReserved2;
    u32 bfOffbits;
};

struct bmp_image_header
{
    //image header
    u32 biSize;
    i32 biWidth;
    i32 biHeight;
    u16 biPlanes;
    u16 biBitCount;
    u32 biCompression;
    u32 biSizeImage;
    u32 biXPelsPerMeter;
    u32 biYPelsPerMeter;
    u32 biClrUsed;
    u32 biClrImportant;
};
#pragma pack(pop)

struct image
{
    u32 *Buffer;
    i32 Width;
    i32 Height;
};

static image 
AllocateImage(i32 Width, i32 Height)
{
    image Result = {};
    Result.Buffer = (u32 *)calloc(Width * Height, sizeof(u32));
    Result.Width = Width;
    Result.Height = Height;
    return Result;
}

static bool 
WriteImageToBMP(image Image, const char *Path)
{
    bool Result = false;
    
    FILE *File = fopen(Path, "wb");
    if (File)
    {
        bmp_file_header BMPFileHeader = {};
        BMPFileHeader.bfType[0] = 'B';
        BMPFileHeader.bfType[1] = 'M';
        BMPFileHeader.bfOffbits = sizeof(bmp_file_header) + sizeof(bmp_image_header);
        
        bmp_image_header BMPImageHeader = {};
        BMPImageHeader.biSize = sizeof(BMPImageHeader);
        BMPImageHeader.biWidth = Image.Width;
        BMPImageHeader.biHeight = Image.Height;
        BMPImageHeader.biPlanes = 1;
        BMPImageHeader.biBitCount = 32;
        
        fwrite(&BMPFileHeader, sizeof(BMPFileHeader), 1, File);
        fwrite(&BMPImageHeader, sizeof(BMPImageHeader), 1, File);
        fwrite(Image.Buffer, sizeof(u32), Image.Width * Image.Height, File);
        fclose(File);
        
        Result = true;
    }
    
    return Result;
}

inline f32 
fract(f32 f)
{
    return fabsf(f - truncf(f));
}

inline f32
Max(float a, float b)
{
    return a > b? a: b;
}

inline f32
Min(float a, float b)
{
    return a < b? a: b;
}

inline f32
Square(f32 X)
{
    return X*X;
}

union vec3
{
    struct
    {
        f32 X, Y, Z;
    };
    
    struct
    {
        f32 R, G, B;
    };
};

struct quaternion
{
    f32 X, Y, Z, W;
};

inline f32
Dot(vec3 a, vec3 b)
{
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
}

inline vec3
Vec3(f32 v)
{
    return {v, v, v};
}

inline vec3
Vec3(f32 x, f32 y, f32 z)
{
    return {x, y, z};
}

inline vec3 
operator-(vec3 a, vec3 b)
{
    vec3 res = {};
    res.X = a.X - b.X;
    res.Y = a.Y - b.Y;
    res.Z = a.Z - b.Z;
    return res;
}

inline vec3 
operator*(vec3 a, f32 s)
{
    vec3 res = a;
    res.X *= s;
    res.Y *= s;
    res.Z *= s;
    return res;
}

inline vec3 
operator*(f32 s, vec3 a)
{
    vec3 res = a;
    res.X *= s;
    res.Y *= s;
    res.Z *= s;
    return res;
}

inline vec3 
operator+(vec3 a, vec3 b)
{
    vec3 res = {};
    res.X = a.X + b.X;
    res.Y = a.Y + b.Y;
    res.Z = a.Z + b.Z;
    return res;
}

inline vec3 
operator/(vec3 &a, f32 s)
{
    vec3 res = a;
    res.X /= s;
    res.Y /= s;
    res.Z /= s;
    return res;
}

inline f32
Length(vec3 v)
{
    return sqrtf(v.X*v.X + v.Y*v.Y + v.Z*v.Z);
}

inline vec3
operator-(vec3 a)
{
    vec3 res;
    res.X = -a.X;
    res.Y = -a.Y;
    res.Z = -a.Z;
    return res;
}

inline vec3
Normalize(vec3 v)
{
    return v / Length(v);
}

inline vec3
Cross(vec3 a, vec3 b)
{
    return {a.Y * b.Z - b.Y * a.Z, a.Z * b.X - b.Z * a.X, a.X * b.Y - b.X * a.Y};
}

inline quaternion
Quat(vec3 p)
{
    quaternion res;
    res.X = p.X;
    res.Y = p.Y;
    res.Z = p.Z;
    res.W = 0.0f;
    return res;
}

inline f32
Length(quaternion q)
{
    return sqrtf(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W);
}

inline quaternion 
operator*(quaternion q1, quaternion q2)
{
    quaternion res;
    res.X = q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y + q1.W*q2.X;
    res.Y = -q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X + q1.W*q2.Y;
    res.Z = q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W + q1.W*q2.Z;
    res.W = q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z;
    return res;
}

inline quaternion 
operator+(quaternion q1, quaternion q2)
{
    quaternion res;
    res.X = q1.X + q2.X;
    res.Y = q1.Y + q2.Y;
    res.Z = q1.Z + q2.Z;
    res.W = q1.W + q2.W;
    return res;
}

inline u32
PackVec3ToU32(vec3 col)
{
    u8 R = (u8)(col.R * 255.0f);
    u8 G = (u8)(col.G * 255.0f);
    u8 B = (u8)(col.B * 255.0f);
    
    return (R << 16 | G << 8 | B);
}

//gives range within [0, 1]
inline f32
RandomUnilateral()
{
    return (f32)rand() / (f32)RAND_MAX;
}

//gives range within [0, 1]
inline f32
RandomBilateral()
{
    return 2.0f * RandomUnilateral() - 1.0f;
}

inline f32
FractalDE(vec3 p)
{
    quaternion Z = {0.0f, p.X, p.Y, p.Z};
    quaternion C = {0.1f, 0.3f, 0.9f, 0.1f};
    
    f32 dZ = 1.0f;
    for (int I = 0; I < 10; ++I)
    {
        dZ = 2.0f * Length(Z) * dZ;
        Z = Z*Z + C;
        
        if (Length(Z) > 256.0f)
        {
            break;
        }
    }
    
    f32 ZMagnitude = Length(Z);
    return 0.5f * ZMagnitude * logf(ZMagnitude) / dZ;
}

inline f32
SceneDE(vec3 P)
{
    return Min(FractalDE(P - Vec3(0.0f, 1.2f, 0.0f)), P.Y);
}

inline vec3
SceneNormal(vec3 P)
{
    vec3 N = Normalize(Vec3(SceneDE(P + Vec3(0.001f, 0.0f, 0.0f)), 
                            SceneDE(P + Vec3(0.0f, 0.001f, 0.0f)), 
                            SceneDE(P + Vec3(0.0f, 0.0f, 0.001f))) - 
                       Vec3(SceneDE(P)));
    return N;
}

inline f32
Shadow(vec3 P, vec3 L)
{
    f32 Res = 1.0f;
    
    f32 T = 0.005f;
    f32 TMax = 10.0f;
    for (int I = 0; I < 256 && T < TMax; ++I)
    {
        float D = SceneDE(P + T*L);
        if (D < 0.0001f)
        {
            return 0.0;
        }
        T += D;
    }
    return Res;
}

static void
RenderImageBadly(image Image)
{
    int SampleCount = 8;
    f32 SampleContrib = 1.0f / (f32)SampleCount;
    f32 PixelHalfWidth = 0.5f / (f32)Image.Width;
    f32 PixelHalfHeight = 0.5f / (f32)Image.Height;
    
    u32 *WriterAddress = Image.Buffer;
    for (int Y = 0; Y < Image.Height; ++Y)
    {
        u32 *Writer = WriterAddress;
        for (int X = 0; X < Image.Width; ++X)
        {
            vec3 FinalColor = {};
            for (int SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
            {
                f32 U = 2.0f * (f32)X / (f32)Image.Width - 1.0f;
                f32 V = 2.0f * (f32)Y / (f32)Image.Height - 1.0f;
                
                //jitter
                U += RandomBilateral() * PixelHalfWidth;
                V += RandomBilateral() * PixelHalfHeight;
                
                U *= (f32)Image.Width / (f32)Image.Height;
                
                vec3 Ro = {2.1f, 1.4f, -2.1f};
                vec3 At = {0.0f, 1.2f, 0.0f};
                vec3 CamZ = Normalize(At - Ro);
                vec3 CamX = Normalize(Cross({0.0f, 1.0f, 0.0f}, CamZ));
                vec3 CamY = Cross(CamZ, CamX);
                vec3 Rd = Normalize(U * CamX + V * CamY + 2.0f * CamZ);
                
                f32 AO = 1.0f;
                int Id = -1;
                f32 T = 0.001f;
                f32 TMax = 30.0f;
                
                for (int I = 0; I < 256 && T < TMax; ++I)
                {
                    vec3 P = Ro + T*Rd;
                    f32 D = SceneDE(P);
                    if (D < 0.001f)
                    {
                        Id = 0;
                        AO = 1.0f - (f32)I / 256.0f;
                        break;
                    }
                    T += D;
                }
                
                vec3 P = Ro + T*Rd;
                vec3 Color = Vec3(0.1f);
                if (Id != -1)
                {
                    vec3 N = SceneNormal(P);
                    vec3 L = -Normalize(Vec3(0.5f, -0.2f, 0.5f));
                    
                    f32 Shad = 0.0f;
                    Shad += 0.7f * Shadow(P, L) * Max(0.0f, Dot(N, L));
                    Shad += AO * 0.3f;
                    Color = Shad * Vec3(0.8f);
                }
                
                FinalColor = FinalColor + SampleContrib * Color;
            }
            *Writer++ = PackVec3ToU32(FinalColor);
        }
        WriterAddress += Image.Width;
        
        f32 Percentage = 100.0f * (f32)Y / (f32)Image.Height;
        printf("\rCompletion: %.2f%%", Percentage);
    }
    printf("\rCompletion: 100.00%%\n");
}

int main()
{
    srand((u32)time(0));
    image Image = AllocateImage(1024, 782);
    
    clock_t T1 = clock();
    RenderImageBadly(Image);
    f32 ElapsedTime = (f32)(clock() - T1) / (f32)CLOCKS_PER_SEC;
    printf("render took %.2f seconds\n", ElapsedTime);
    
    bool WriteResult = WriteImageToBMP(Image, "fractal.bmp");
    return 0;
}

