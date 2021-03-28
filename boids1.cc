#include <iostream>
#include <random>
#include <vector>

// argument parsing
#define SOKOL_IMPL
#include "sokol_args.h"

// gfx
extern "C" {
    #include <raylib.h>
}

// for vectors
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>
#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#define BACKGROUND_COLOR (Color) { 125, 132, 178, 255 }
#define BOID_COLOR (Color) { 219, 244, 167, 255 }

static std::string TrueFalseArray[] = { "false", "true" };
static glm::vec2 SimulationBounds = glm::vec2(1920, 1080);
static Texture2D drawTexture;

class Boid
{
public:
    Boid(float x, float y, float angle) : velocity(cos(angle), sin(angle)), position(x, y)
    {
        velocity = glm::normalize(velocity);
        velocity *= MaxSpeed;

        // id stuff
        ID = boid_count;
        boid_count++;
    }

    void Update(std::vector<Boid*>& boids)
    {
        glm::vec2 avoid = AvoidOtherBoids(boids) * 3.0f;
        glm::vec2 align = AlignWithOtherBoids(boids) * 1.0f;
        glm::vec2 cohere = CohereWithOtherBoids(boids) * 5.0f;
        glm::vec2 bounds = ConfineToBounds() * 1.0f;
        ApplyForce(avoid + align + cohere + bounds);

        velocity += acceleration;
        velocity = glm::normalize(velocity) * MaxSpeed;
        position += velocity;
        acceleration *= 0;

        // WrapAround(SimulationBounds.x, SimulationBounds.y);
    }

    void Update(std::vector<Boid*>& boids, glm::vec2 risk)
    {
        glm::vec2 avoid = AvoidOtherBoids(boids) * 2.0f;
        glm::vec2 align = AlignWithOtherBoids(boids) * 1.0f;
        glm::vec2 cohere = CohereWithOtherBoids(boids) * 2.0f;
        glm::vec2 bounds = ConfineToBounds() * 5.0f;
        glm::vec2 seek_away = seek_to(risk) * -5.0f;
        ApplyForce(avoid + align + cohere + bounds + seek_away);

        ApplyForce(avoid + align + cohere + seek_away);

        velocity += acceleration;
        velocity = glm::normalize(velocity) * MaxSpeed;
        position += velocity;
        acceleration *= 0;

        // WrapAround(SimulationBounds.x, SimulationBounds.y);
    }

    void Draw()
    {
        // DrawCircleV({ position.x, position.y }, 2.0f, BLACK);
        // DrawCircle(position.x, position.y, NearbyValue, { 255, 0, 0, 32 });
        
        #define RADIANS_TO_DEGREES(r) (r * (180 / PI))
        #define DEGREES_TO_RADIANS(d) (d * (PI / 180))

        float angle_degrees = abs(RADIANS_TO_DEGREES(std::atan2(velocity.y, velocity.x))) + 90;
        DrawTexturePro(drawTexture,
                    { 0, 0, (float) drawTexture.width, (float) drawTexture.height },
                    { position.x, position.y, (float) drawTexture.width * 2, (float) drawTexture.height * 2 },
                    { 4, 4 },
                    angle_degrees,
                    WHITE);
    }

    void ApplyForce(glm::vec2 force)
    {
        acceleration += force;
    }

    glm::vec2 GetPos() { return position; }
    void SetPos(glm::vec2 pos) { position = pos; }
    void AddPos(float x, float y) { position += glm::vec2(x, y); }
    glm::vec2 GetVelocity() { return velocity; }
    unsigned int GetID() { return ID; }
private: // functions
    glm::vec2 AvoidOtherBoids(std::vector<Boid*>& boids) //  Rule 1
    {
        glm::vec2 steer;
        float count = 0;

        for (auto boid : boids)
        {
            if (ID != boid->GetID())
            {
                float distance = abs(glm::length(position - boid->GetPos()));
                
                if (distance <= DesiredSeparation) // if within bounds then separate
                {
                    glm::vec2 difference = position - boid->GetPos();
                    difference = glm::normalize(difference);
                    difference /= distance;
                    steer += difference;
                    count++;
                }
            }
        }
        
        if (count > 0) steer /= count;
       
        if (glm::length(steer) > 0)
        {
            steer = glm::normalize(steer);
            steer *= MaxSpeed;
            steer -= velocity;
            steer = glm::normalize(steer) * MaxSteerForce;
        }
        return steer;
    }

    glm::vec2 AlignWithOtherBoids(std::vector<Boid*>& boids) // Rule 2
    {
        glm::vec2 sum;
        float count = 0;
        
        for (auto boid : boids)
        {
            if (ID != boid->GetID())
            {
                float distance = abs(glm::length(position - boid->GetPos()));
                
                if (distance < NearbyValue)
                {
                    sum += boid->GetVelocity();
                    count++;
                }
            }
        }

        if (count > 0)
        {
            sum /= count;

            sum = glm::normalize(sum);
            sum *= MaxSpeed;

            glm::vec2 steer = sum - velocity;
            steer = glm::normalize(steer) * MaxSteerForce;
            return steer;
        }
        else
        {
            return glm::vec2();
        }
    }

    glm::vec2 CohereWithOtherBoids(std::vector<Boid*>& boids) // Rule 3
    {
        glm::vec2 sum;
        float count = 0;

        for (auto boid : boids)
        {
            if (ID != boid->GetID())
            {
                float distance = abs(glm::length(position - boid->GetPos()));
                if (distance < NearbyValue)
                {
                    if (DrawLineOfSight) DrawLine(position.x, position.y, boid->GetPos().x, boid->GetPos().y, RED);
                    sum += boid->GetPos();
                    count++;
                }
            }
        }

        if (count > 0)
        {
            sum /= count;
            if (DrawAveragePosition) DrawCircle(sum.x, sum.y, 2.0f, BLUE);
            return seek_to(sum);
        }
        else
        {
            return glm::vec2(0, 0);
        }
    }

    glm::vec2 ConfineToBounds()
    {
        glm::vec2 steer;

        if (position.x < 0)
        {
            steer.x = 10;
        }
		else if (position.x > SimulationBounds.x)
        {
            steer.x = -10;
        }
        else if (position.y < 0)
        {
            steer.y = 10;
        }
        else if (position.y > SimulationBounds.y)
        {
            steer.y = -10;
        }
		
        if (glm::length(steer) > 0)
        {
            steer = glm::normalize(steer);
            steer *= MaxSpeed;
            steer -= velocity;
            steer = glm::normalize(steer) * MaxSteerForce;
        }

        return steer;
    }
public: // config
    // physics options
    inline static float MaxSpeed = 3.0f;
    inline static float MaxSteerForce = 0.04f;
    inline static float DesiredSeparation = 128.0f;
    inline static float NearbyValue = 256.0f;

    // visual help config
    inline static bool DrawLineOfSight = false;
    inline static bool DrawAveragePosition = false;
private: // helpers
    glm::vec2 seek_to(glm::vec2 target)
    {
        glm::vec2 desired = target - position;

        desired = glm::normalize(desired);
        desired *= MaxSpeed;

        glm::vec2 steer = desired - velocity;
        steer = glm::normalize(steer) * MaxSteerForce;
      
        return steer;
    }
private: // fields
    glm::vec2 acceleration;
    glm::vec2 velocity;
    glm::vec2 position;
private: // ID stuff
    unsigned int ID;
    inline static int boid_count = 0;
};

class BoidFlock
{
public:
    BoidFlock(unsigned int count)
    {
        CreateBoids(count);
    }

    BoidFlock(glm::vec2 origin, unsigned int count)
    {
        std::random_device rnd;
        std::mt19937 gen;

        std::uniform_real_distribution<float> angle_rand(0, 2 * PI);

        for (int i = 0; i < count; i++)
        {
            boids.push_back(new Boid(origin.x, origin.y, angle_rand(gen)));
        }
    }

    ~BoidFlock()
    {
        Clear();
    }

    void CreateBoids(unsigned int count)
    {
        std::random_device rnd;
        std::mt19937 gen;

        std::uniform_int_distribution<> x_rand(0, SimulationBounds.x);
        std::uniform_int_distribution<> y_rand(0, SimulationBounds.y);
        std::uniform_real_distribution<float> angle_rand(0, 2 * PI);

        for (int i = 0; i < count; i++)
        {
            boids.push_back(new Boid(x_rand(gen), y_rand(gen), angle_rand(gen)));
        }
    }

    void Clear()
    {
        for (auto boid : boids)
        {
            delete(boid);
        }

        boids.empty();
        boids.shrink_to_fit();
    }

    void Update()
    {
        for (auto boid : boids)
        {
            
            if (IsKeyDown(KEY_LEFT_SHIFT)) boid->ApplyForce(glm::vec2(IsKeyDown(KEY_RIGHT) * 10.0f + IsKeyDown(KEY_LEFT) * -10.0f, IsKeyDown(KEY_UP) * -10.0f + IsKeyDown(KEY_DOWN) * 10.0f));
            if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) boid->Update(boids, glm::vec2(GetMouseX(), GetMouseY()));
            else boid->Update(boids);
        }
    }

    void Draw()
    {
        for (auto boid : boids)
        {
            boid->Draw();
        }
    }

    Boid* AddBoid(float x, float y)
    {
        Boid* boid = new Boid(x, y, 0);
        boids.push_back(boid);

        return boid;
    }
public: // config
    inline static int InitialBoids = 64;
private: // fields
    std::vector<Boid*> boids;
};

void GetArguments(int argc, char* argv[])
{
    sargs_desc desc = { .argc = argc, .argv = argv };
    sargs_setup(&desc);
    if (sargs_exists("initial_boids"))
    {
        try
        {
            BoidFlock::InitialBoids = std::stoi(sargs_value("initial_boids"));;
        }
        catch(const std::exception& e)
        {
            std::cout << "initial_boids was set to an invalid value \"" << sargs_value("initial_boids") << "\"" << std::endl;
            BoidFlock::InitialBoids = 64;
        }
    }

    sargs_shutdown();
}

int main(int argc, char* argv[])
{
    GetArguments(argc, argv);

    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);

    InitWindow(SimulationBounds.x, SimulationBounds.y, "boids");
    SetTargetFPS(60);

    drawTexture = LoadTexture("triangle.png");

    BoidFlock* flock = new BoidFlock(BoidFlock::InitialBoids);

    bool new_boid_queued = false;
    float new_boid_x = 0, new_boid_y = 0;
    while (!WindowShouldClose())
    {
        //TODO: Do runtime reload thingie

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            // set vars
            new_boid_x = GetMouseX();
            new_boid_y = GetMouseY();

            new_boid_queued = true;
        }

        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
        {
            // add boid
            Boid* new_boid = flock->AddBoid(new_boid_x, new_boid_y);
            // calculate + add velocity
            new_boid->ApplyForce(glm::vec2(GetMouseX(), GetMouseY()) - new_boid->GetPos());

            // reset vars
            new_boid_queued = false;

            new_boid_x = 0.0f;
            new_boid_y = 0.0f;
        }

        if (IsWindowResized())
        {
            SimulationBounds.x = GetScreenWidth();
            SimulationBounds.y = GetScreenHeight();
        }

        BeginDrawing();
            ClearBackground(BACKGROUND_COLOR);
            flock->Update();
            flock->Draw();

            if (new_boid_queued)
            {
                DrawCircle(new_boid_x, new_boid_y, 4.0f, PURPLE);
                DrawLine(GetMouseX(), GetMouseY(), new_boid_x, new_boid_y, RED);
            }
            DrawFPS(0, 0);
        EndDrawing();
    }

    delete(flock);

    UnloadTexture(drawTexture);
    CloseWindow();
}
