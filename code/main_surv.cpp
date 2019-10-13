#include "SDL.h"
#undef main
#include <iostream>
#include "glad.h"
#include "math_surv.h"
#include "common_surv.h"
#include "render_surv.h"

#define SecondsPerFrame 1.0f / 60.0f
#define PixelsPerMeter 100.0f

u64 PlatformCycleCount;

const char MainVertexShader[] =
{
    "#version 330 core\n"
    "layout (location = 0) in vec2 aPos;\n"
    "layout (location = 1) in vec4 aColor;\n"
    "uniform mat4 ProjView;\n"
    "out vec4 OutVertexColor;\n"
    "void main()\n"
    "{\n"
        "OutVertexColor = aColor;\n"
        "gl_Position = ProjView * vec4(aPos.x, aPos.y, 0.0f, 1.0);\n"
    "}\n"
};

const char MainFragmentShader[] =
{
    "#version 330 core\n"
    "in vec4 OutVertexColor;\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "    FragColor = OutVertexColor;\n"
    "}\n" 
};

#define MaxGameThings 4096
struct game_things
{
    f32 X[MaxGameThings];
    f32 Y[MaxGameThings];
    f32 InitVelX[MaxGameThings];
    f32 InitVelY[MaxGameThings];
    f32 VelocityX[MaxGameThings];
    f32 VelocityY[MaxGameThings];
    f32 Speed[MaxGameThings];
    f32 Radius[MaxGameThings];
    f32 ColorR[MaxGameThings];
    f32 ColorG[MaxGameThings];
    f32 ColorB[MaxGameThings];
    f32 ColorA[MaxGameThings];
    b32 Immobile[MaxGameThings];
    b32 Collideable[MaxGameThings];
    b32 Colliding[MaxGameThings];
    //Inactive game things are removed after they are rendered
    b32 Active[MaxGameThings];
    i32 NumberOfThings;
};

game_things GameThings = {};

i32 PlayerIndex = 0;
i32 SpeedyThing = 0;

#define TOGGLE_PAUSE 1
#define TOGGLE_NEXT_FRAME 2
struct game_input
{
    b32 MoveUp;
    b32 MoveLeft;
    b32 MoveDown;
    b32 MoveRight;
    b32 Extra[16];
};

game_input GameInput = {};
game_input PreviousGameInput = {};

b32 GamePaused = 0;
b32 GameRunning = 1;

f32 GetSecondsElapsed(u64 PrevCounter, u64 CurrentCounter)
{
    return ((f32) (CurrentCounter - PrevCounter)) / PlatformCycleCount;
}

u64 GetHardwareTime()
{
    return SDL_GetPerformanceCounter();
}

void SwapGameThings(i32 Thing1, i32 Thing2)
{
    f32 X = GameThings.X[Thing1];
    f32 Y = GameThings.Y[Thing1];
    f32 VelocityX = GameThings.VelocityX[Thing1];
    f32 VelocityY = GameThings.VelocityY[Thing1];
    f32 InitVelX = GameThings.InitVelX[Thing1];
    f32 InitVelY = GameThings.InitVelX[Thing1];
    f32 Speed = GameThings.Speed[Thing1];
    f32 Radius = GameThings.Radius[Thing1];
    f32 ColorR = GameThings.ColorR[Thing1];
    f32 ColorG = GameThings.ColorG[Thing1];
    f32 ColorB = GameThings.ColorB[Thing1];
    f32 ColorA = GameThings.ColorA[Thing1];
    f32 Immobile = GameThings.Immobile[Thing1];
    b32 Collideable = GameThings.Collideable[Thing1];
    b32 Colliding = GameThings.Colliding[Thing1];
    b32 Active = GameThings.Active[Thing1];

    GameThings.X[Thing1] = GameThings.X[Thing2];
    GameThings.Y[Thing1] = GameThings.Y[Thing2];
    GameThings.InitVelX[Thing1] = GameThings.InitVelX[Thing2];
    GameThings.InitVelY[Thing1] = GameThings.InitVelY[Thing2];
    GameThings.VelocityX[Thing1] = GameThings.VelocityX[Thing2];
    GameThings.VelocityY[Thing1] = GameThings.VelocityY[Thing2];
    GameThings.Speed[Thing1] = GameThings.Speed[Thing2];
    GameThings.Radius[Thing1] = GameThings.Radius[Thing2];
    GameThings.ColorR[Thing1] = GameThings.ColorR[Thing2];
    GameThings.ColorG[Thing1] = GameThings.ColorG[Thing2];
    GameThings.ColorB[Thing1] = GameThings.ColorB[Thing2];
    GameThings.ColorA[Thing1] = GameThings.ColorA[Thing2];
    GameThings.Immobile[Thing1] = GameThings.Immobile[Thing2];
    GameThings.Collideable[Thing1] = GameThings.Collideable[Thing2];
    GameThings.Colliding[Thing1] = GameThings.Colliding[Thing2];
    GameThings.Active[Thing1] = GameThings.Active[Thing2];
    
    GameThings.X[Thing2] = X;
    GameThings.Y[Thing2] = Y;
    GameThings.InitVelX[Thing2] = InitVelX;
    GameThings.InitVelY[Thing2] = InitVelY;
    GameThings.VelocityX[Thing2] = VelocityX;
    GameThings.VelocityY[Thing2] = VelocityY;
    GameThings.Speed[Thing2] = Speed;
    GameThings.Radius[Thing2] = Radius;
    GameThings.ColorR[Thing2] = ColorR;
    GameThings.ColorG[Thing2] = ColorG;
    GameThings.ColorB[Thing2] = ColorB;
    GameThings.ColorA[Thing2] = ColorA;
    GameThings.Immobile[Thing2] = Immobile;
    GameThings.Collideable[Thing1] = Collideable;
    GameThings.Colliding[Thing1] = Colliding;
    GameThings.Active[Thing1] = Active;
}

b32 ResolveAnyCollision(i32 Thing1, i32 Thing2, f32 FrameTime)
{
    f32 CollisionEpsilon = 1 / 2048.0f;
    //with sum radius, relative position and velocity, we turn circle-circle overlap test into circle-line overlap test

    //summing radius
    f32 SumRadius = GameThings.Radius[Thing1] + GameThings.Radius[Thing2];
    
    //position of thing 1 relative to thing 2. this is the center of the summed circle
    f32 CenterX = GameThings.X[Thing1] - GameThings.X[Thing2];
    f32 CenterY = GameThings.Y[Thing1] - GameThings.Y[Thing2];
    
    //velocity of thing 2 relative to thing 1. only care if a collision will happen within this frame
    f32 VelX = (GameThings.VelocityX[Thing2] - GameThings.VelocityX[Thing1]);
    f32 VelY = (GameThings.VelocityY[Thing2] - GameThings.VelocityY[Thing1]);
    
    f32 VelMag = Sqrt(VelX * VelX + VelY * VelY);
    f32 VelMagFrame = VelMag * FrameTime;
    f32 CenterToOriginSq = CenterX * CenterX + CenterY * CenterY;
    if(VelMagFrame > CollisionEpsilon) //ray-circle intersection
    {
        f32 RayX = 0;
        f32 RayY = 0;
        f32 RayDirX = VelX * FrameTime;
        f32 RayDirY = VelY * FrameTime;
        f32 DiffX = RayX - CenterX;
        f32 DiffY = RayX - CenterY;
        f32 DotDirDiff = (RayDirX * DiffX) + (RayDirY * DiffY);
        f32 InsideSqrt = DotDirDiff * DotDirDiff - ((DiffX * DiffX + DiffY * DiffY) - SumRadius * SumRadius);
        if(InsideSqrt < 0 || (CenterToOriginSq > SumRadius * SumRadius && DotDirDiff >= 0))
        {
            return 0; 
        }
        f32 SqrtValue = Sqrt(InsideSqrt);
        f32 ResolveScalar1 = -DotDirDiff - SqrtValue;
        f32 ResolveScalar2= -DotDirDiff + SqrtValue;
        v2 Resolve1 = v2{VelX, VelY} * (ResolveScalar1);
        v2 Resolve2 = v2{VelX, VelY} * (ResolveScalar2);
        v2 Resolve = Resolve1;
        f32 DotResolveDiff= Resolve.X * DiffX + Resolve.Y * DiffY;
        //printf("Res1 %f Res2 %f VelX %f VelY %f CenterX %f CenterY %f Dot\n", ResolveScalar1, ResolveScalar2, VelX, VelY, CenterX, CenterY);
        if(DotResolveDiff < 0)
        {
            Resolve = Resolve2;
        }

        Resolve = Resolve * FrameTime;
        
        GameThings.X[Thing1] -= Resolve.X * 0.5f;
        GameThings.Y[Thing1] -= Resolve.Y * 0.5f;
        GameThings.X[Thing2] += Resolve.X * 0.5f;
        GameThings.Y[Thing2] += Resolve.Y * 0.5f;

        GameThings.VelocityX[Thing1] = 0;
        GameThings.VelocityY[Thing1] = 0;
        GameThings.VelocityX[Thing2] = 0;
        GameThings.VelocityY[Thing2] = 0;
    }
    else //point-circle intersection
    {
        if(CenterToOriginSq > SumRadius * SumRadius)
        {
            return 0; 
        }
        f32 CenterMag = Sqrt(CenterToOriginSq);
        v2 CenterToOriginUnit = {CenterX / CenterMag, CenterY / CenterMag};
        f32 MinSeparationDist =  SumRadius - CenterMag + CollisionEpsilon;
        v2 Separation = 
        {
            CenterToOriginUnit.X * MinSeparationDist,
            CenterToOriginUnit.Y * MinSeparationDist,
        };

        GameThings.X[Thing1] += Separation.X * 0.5f;
        GameThings.Y[Thing1] += Separation.Y * 0.5f;
        GameThings.X[Thing2] -= Separation.X * 0.5f;
        GameThings.Y[Thing2] -= Separation.Y * 0.5f;
    }

    return 1;
}

i32 AddGameThing()
{
    Assert(GameThings.NumberOfThings+1 < MaxGameThings);
    i32 Index = GameThings.NumberOfThings++;
    GameThings.Active[Index] = 1; 
    return Index;
}

void UseCollideColors(i32 Thing)
{
    GameThings.ColorR[Thing] = 1;
    GameThings.ColorG[Thing] = 0;
    GameThings.ColorB[Thing] = 0;
    GameThings.ColorA[Thing] = 1;
}

void UseNonCollideColors(i32 Thing)
{
    GameThings.ColorR[Thing] = 1;
    GameThings.ColorG[Thing] = 1;
    GameThings.ColorB[Thing] = 1;
    GameThings.ColorA[Thing] = 1;
}

void RunFrame(f32 FrameTime)
{
    Assert(GameThings.NumberOfThings < MaxGameThings);

    GameThings.InitVelX[PlayerIndex] = 0;
    GameThings.InitVelY[PlayerIndex] = 0;
    GameThings.InitVelX[PlayerIndex] += (f32) GameInput.MoveRight != 0;
    GameThings.InitVelX[PlayerIndex] -= (f32) GameInput.MoveLeft != 0;
    GameThings.InitVelY[PlayerIndex] += (f32) GameInput.MoveUp != 0;
    GameThings.InitVelY[PlayerIndex] -= (f32) GameInput.MoveDown != 0;
    f32 Magnitude = Sqrt(GameThings.InitVelX[PlayerIndex] * GameThings.InitVelX[PlayerIndex] + GameThings.InitVelY[PlayerIndex] * GameThings.InitVelY[PlayerIndex]);
    if(Magnitude > 0)
    {
        GameThings.InitVelX[PlayerIndex] /= Magnitude;
        GameThings.InitVelY[PlayerIndex] /= Magnitude;
    }
    GameThings.InitVelX[PlayerIndex] *= GameThings.Speed[PlayerIndex];
    GameThings.InitVelY[PlayerIndex] *= GameThings.Speed[PlayerIndex];

    if(GameInput.Extra[0] && !PreviousGameInput.Extra[0])
    {
        GameThings.InitVelX[SpeedyThing] = GameThings.Speed[SpeedyThing];
        GameThings.X[SpeedyThing] = -1.0f;
        GameThings.Y[SpeedyThing] = GameThings.Y[PlayerIndex];
    }

    for(i32 Index = 0; Index < GameThings.NumberOfThings; Index++)
    {
        if(GameThings.Collideable)
        {
            for(i32 Index2 = Index+1; Index2 < GameThings.NumberOfThings; Index2++)
            {
                if(GameThings.Collideable == 0)
                {
                    continue;
                }
                b32 Colliding = ResolveAnyCollision(Index, Index2, FrameTime);
                GameThings.Colliding[Index] = Colliding || GameThings.Colliding[Index];
                GameThings.Colliding[Index2] = Colliding || GameThings.Colliding[Index2];
            }
        }
        if(GameThings.Colliding[Index])
        {
            UseCollideColors(Index);
        }
        else
        {
            UseNonCollideColors(Index);
        }

        if(!GamePaused || GameInput.Extra[TOGGLE_NEXT_FRAME] && !PreviousGameInput.Extra[TOGGLE_NEXT_FRAME])
        {
            GameThings.X[Index] += GameThings.VelocityX[Index] * FrameTime;
            GameThings.Y[Index] += GameThings.VelocityY[Index] * FrameTime;

        }
    }
    if(!PreviousGameInput.Extra[TOGGLE_PAUSE] && GameInput.Extra[TOGGLE_PAUSE])
    {
        GamePaused = !GamePaused;
    }

    StartRenderBatch();
    #define CircleRenderSegments 360
    for(i32 Counter = GameThings.NumberOfThings - 1; Counter >= 0; Counter--) {
        RenderCircle(GameThings.X[Counter] * PixelsPerMeter, GameThings.Y[Counter] * PixelsPerMeter, GameThings.Radius[Counter] * PixelsPerMeter, GameThings.ColorR[Counter], GameThings.ColorG[Counter], GameThings.ColorB[Counter], GameThings.ColorA[Counter], CircleRenderSegments);
        GameThings.Colliding[Counter] = 0;
        GameThings.VelocityX[Counter] = GameThings.InitVelX[Counter];
        GameThings.VelocityY[Counter] = GameThings.InitVelY[Counter];
    }
    FinishRenderBatch();
}

int main()
{
    #if defined (_WIN32) || defined (__WIN32) || defined(WIN32) || defined(_MSC_VER)
    i32 ScreenWidth = (i32) GetSystemMetrics(SM_CXSCREEN);
    i32 ScreenHeight = (i32) GetSystemMetrics(SM_CYSCREEN);
    #else
    SDL_DisplayMode Display;
    SDL_GetDesktopDisplayMode(0, &Display);
    i32 ScreenWidth = Display.w;
    i32 ScreenHeight= Display.h;
    #endif

    SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    PrecomputeTrig();

    if(SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* Window = SDL_CreateWindow("survival", SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED, ScreenWidth, ScreenHeight, SDL_WINDOW_BORDERLESS | SDL_WINDOW_OPENGL);
    if (!Window)
    {
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
    SDL_GLContext GLContext = SDL_GL_CreateContext(Window);
    SDL_GL_SetSwapInterval(1);

    if(!gladLoadGLLoader(SDL_GL_GetProcAddress))
    {
        std::cout << "Couldn't Load GL functions" << std::endl;
        SDL_DestroyWindow(Window);
        SDL_Quit();
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    PlatformCycleCount = SDL_GetPerformanceFrequency();

    GLuint MainShaderProgram = CreateVFShader(MainVertexShader, MainFragmentShader);
    glUseProgram(MainShaderProgram);
    mat4 Projection = OrthographicMatrix(0, (f32) ScreenWidth, 0, (f32) ScreenHeight, -1.0f, 100.0f);

    PlayerIndex = AddGameThing();
    GameThings.X[PlayerIndex] = 1;
    GameThings.Y[PlayerIndex] = 1;
    GameThings.Radius[PlayerIndex] = 1;
    GameThings.ColorR[PlayerIndex] = 1;
    GameThings.ColorG[PlayerIndex] = 0;
    GameThings.ColorB[PlayerIndex] = 0;
    GameThings.ColorA[PlayerIndex] = 1;
    GameThings.Immobile[PlayerIndex] = 0;
    GameThings.Speed[PlayerIndex] = 2;
    GameThings.Collideable[PlayerIndex] = 1;

    i32 SampleThing = AddGameThing();
    GameThings.X[SampleThing] = 7;
    GameThings.Y[SampleThing] = 4;
    GameThings.Radius[SampleThing] = 1;
    GameThings.ColorR[SampleThing] = 0.5f;
    GameThings.ColorG[SampleThing] = 1;
    GameThings.ColorB[SampleThing] = 1;
    GameThings.ColorA[SampleThing] = 1;
    GameThings.Immobile[SampleThing] = 1;
    GameThings.Collideable[SampleThing] = 1;

    SampleThing = AddGameThing();
    GameThings.X[SampleThing] = 7;
    GameThings.Y[SampleThing] = 6.5f;
    GameThings.Radius[SampleThing] = 1;
    GameThings.ColorR[SampleThing] = 1;
    GameThings.ColorG[SampleThing] = 0.5f;
    GameThings.ColorB[SampleThing] = 1;
    GameThings.ColorA[SampleThing] = 1;
    GameThings.Immobile[SampleThing] = 1;
    GameThings.Collideable[SampleThing] = 1;

    SpeedyThing = AddGameThing();
    GameThings.Radius[SpeedyThing] = 1;
    GameThings.ColorR[SpeedyThing] = 0.5f;
    GameThings.ColorG[SpeedyThing] = 1;
    GameThings.ColorB[SpeedyThing] = 0;
    GameThings.ColorA[SpeedyThing] = 1;
    GameThings.Immobile[SpeedyThing] = 0;
    GameThings.Speed[SpeedyThing] = 100;
    GameThings.Collideable[SpeedyThing] = 1;

    while(GameRunning)
    {
        u64 StartCounter = SDL_GetPerformanceCounter();
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        SDL_PumpEvents();
        const u8* KeyStates = SDL_GetKeyboardState(0);
        GameRunning = !(KeyStates[SDL_SCANCODE_LALT] && KeyStates[SDL_SCANCODE_F4]);
        GameInput.MoveUp =    KeyStates[SDL_SCANCODE_W] || KeyStates[SDL_SCANCODE_UP];
        GameInput.MoveLeft =  KeyStates[SDL_SCANCODE_A] || KeyStates[SDL_SCANCODE_LEFT];
        GameInput.MoveDown =  KeyStates[SDL_SCANCODE_S] || KeyStates[SDL_SCANCODE_DOWN];
        GameInput.MoveRight = KeyStates[SDL_SCANCODE_D] || KeyStates[SDL_SCANCODE_RIGHT];
        GameInput.Extra[0] = KeyStates[SDL_SCANCODE_V];
        GameInput.Extra[TOGGLE_PAUSE] = KeyStates[SDL_SCANCODE_P];
        GameInput.Extra[TOGGLE_NEXT_FRAME] = KeyStates[SDL_SCANCODE_N];

        mat4 ProjViewMat = Projection;
        GLint ProjViewLoc = glGetUniformLocation(MainShaderProgram, "ProjView");
        glUniformMatrix4fv(ProjViewLoc, 1, GL_TRUE, ProjViewMat.M);

        RunFrame(SecondsPerFrame);
        PreviousGameInput = GameInput;

        //swap window after done rendering
        SDL_GL_SwapWindow(Window);

        f32 ElapsedSeconds = GetSecondsElapsed(StartCounter, SDL_GetPerformanceCounter());
        if(ElapsedSeconds < SecondsPerFrame)
        {
            i32 SleepMS = (i32) (1000 * (SecondsPerFrame - ElapsedSeconds)) - 1;
            if(SleepMS > 0)
                SDL_Delay(SleepMS);
            do 
            {
                ElapsedSeconds = GetSecondsElapsed(StartCounter, SDL_GetPerformanceCounter());
            } while(ElapsedSeconds < SecondsPerFrame);
        }
    }
}
