#define IM_VEC2_CLASS_EXTRA
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#define IMAPP_IMPL
#include "ImApp.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include "imgui_internal.h"

#define MAX_NEIGHBOURS 6

struct Hexagrid
{
    void Init(int sideSize, int seed, int searchIterationCount)
    {
        mSearchIterationCount = searchIterationCount;
        mPoints.clear();
        mTriangles.clear();
        mQuads.clear();
        mNeighbours.clear();

        if (sideSize < 2)
        {
            return;
        }
        mSideSize = sideSize;

        mPoints.clear();
        mTriangles.clear();
        mQuads.clear();
        static const float sideLength = 0.5f * tanf(3.141592f * 0.166666f * 2.f); // 0.5f * tanf(60deg)
        for (int x = 0; x < (sideSize * 2 - 1) ; x++)
        {
            int height = (x < sideSize) ? (sideSize + x) : (sideSize * 3 - 2 - x);
            float deltaHeight = sideSize - height * 0.5f;
            //deltaHeight = 0.f;
            for (int y = 0; y < height; y++)
            {
                bool isSide = x == 0 || x == (sideSize * 2 - 2) || y == 0 || y == height-1;
                mPoints.push_back({ImVec2((x - sideSize + 1) * sideLength, y + deltaHeight), isSide});
            }
        }
        int offset = 0;
        for (int x = 0; x < (sideSize * 2 - 2); x++)
        {
            int height = (x < sideSize) ? (sideSize + x) : (sideSize * 3 - 2 - x);
            if (x < sideSize - 1)
            {
                // left side
                for (int y = 0; y < height; y++)
                {
                    mTriangles.push_back({ offset + y, offset + y + height, offset + y + height + 1 });
                    if (y >= height - 1)
                    {
                        break;
                    }
                    mTriangles.push_back({ offset + y + height + 1, offset + y + 1, offset + y });
                }
            }
            else
            {
                // right side
                for (int y = 0; y < height - 1; y++)
                {
                    mTriangles.push_back({ offset + y, offset + y + height, offset + y + 1 });
                    if (y >= height - 2)
                    {
                        break;
                    }
                    mTriangles.push_back({ offset + y + 1, offset + y + height, offset + y + height + 1 });
                }
            }
            offset += height;
        }
        // triangles to quads

        int triIndex;
        int adjacents[3];
        
        srand(seed);
        while(1)
        {
            int searchCount = 0;
            do
            {
                triIndex = rand() % mTriangles.size();
                searchCount++;
            }
            while(searchCount < mSearchIterationCount && !mTriangles[triIndex].mValid);
            if (searchCount == mSearchIterationCount)
            {
                break;
            }

            int adjacentCount = GetAdjacentTriangles(triIndex, adjacents);
            if (adjacentCount > 0)
            {
                int i1 = triIndex;
                int i2 = adjacents[0];
                int indices[6] = { mTriangles[i1].mA, mTriangles[i1].mB, mTriangles[i1].mC,
                    mTriangles[i2].mA, mTriangles[i2].mB, mTriangles[i2].mC };
                std::sort(&indices[0], &indices[6]);
                int quadIndices[4];
                int quadIndexCount = 1;
                quadIndices[0] = indices[0];
                for (int i = 1; i < 6; i++)
                {
                    if (indices[i] != indices[i-1])
                    {
                        quadIndices[quadIndexCount++] = indices[i];
                    }
                }
                assert(quadIndexCount == 4);

                mQuads.push_back({ quadIndices[0], quadIndices[2], quadIndices[3], quadIndices[1]});
                mTriangles[triIndex].mValid = false;;
                mTriangles[adjacents[0]].mValid = false;
            }
        }

        std::map<uint32_t, int> middles;
        // quads to 4 quads
        mBaseQuadCount = (int)mQuads.size();
        for (int i = 0; i < mBaseQuadCount; i++)
        {
            auto quad = mQuads[i];
            const int* index = &quad.mA;
            int indexCenter = (int)mPoints.size();
            mPoints.push_back({(mPoints[quad.mA].mPosition + mPoints[quad.mB].mPosition + mPoints[quad.mC].mPosition + mPoints[quad.mD].mPosition) * 0.25f});
            Subdivide<4>(index, middles, indexCenter);
        }

        // triangles to quads
        for(const auto& triangle : mTriangles)
        {
            if (triangle.mValid)
            {
                const int* index = &triangle.mA;
                int indexCenter = (int)mPoints.size();
                mPoints.push_back({(mPoints[triangle.mA].mPosition + mPoints[triangle.mB].mPosition + mPoints[triangle.mC].mPosition) * 0.3333f});
                Subdivide<3>(index, middles, indexCenter);
            }
        }
        
        // neighbours
        mNeighbours.resize(mPoints.size());
        for (size_t i = mBaseQuadCount; i < mQuads.size(); i++)
        {
            auto quad = mQuads[i];
            int* index = &quad.mA;
            for (int j = 0; j < 4; j++)
            {
                int index1 = index[j];
                int index2 = index[ (j + 1) & 3];
                {
                    auto& neighbour = mNeighbours[index1];
                    // check
                    bool good = true;
                    for (int k = 0; k < neighbour.mNeighbourCount; k++)
                    {
                        if(neighbour.mNeighbour[k] == index2)
                        {
                            good=false;
                            break;
                        }
                    }
                    if (good)
                    {
                        assert(neighbour.mNeighbourCount < MAX_NEIGHBOURS);
                        neighbour.mNeighbour[neighbour.mNeighbourCount++] = index2;
                    }
                }
                {
                    auto& neighbour = mNeighbours[index2];
                    // check
                    bool good = true;
                    for (int k = 0; k < neighbour.mNeighbourCount; k++)
                    {
                        if(neighbour.mNeighbour[k] == index1)
                        {
                            good=false;
                            break;
                        }
                    }
                    if (good)
                    {
                        assert(neighbour.mNeighbourCount < MAX_NEIGHBOURS);
                        neighbour.mNeighbour[neighbour.mNeighbourCount++] = index1;
                    }
                }
            }
        }
    }

    void Relax()
    {
        for (int i = 0; i < mPoints.size(); i++)
        {
            if (mPoints[i].mSide)
            {
                continue;
            }
            const auto& neighbour = mNeighbours[i];
            ImVec2 sum(0.f, 0.f);
            for (int j = 0; j < neighbour.mNeighbourCount; j++)
            {
                sum += mPoints[neighbour.mNeighbour[j]].mPosition;
            }
            sum /= (float)neighbour.mNeighbourCount;
            mPoints[i].mPosition = sum;
        }
    }

    void RelaxSide()
    {
        const float radius = mSideSize - 1.f;
        ImVec2 center(0.f, (mSideSize * 2 - 1) * 0.5f);

        for (int i = 0; i < mPoints.size(); i++)
        {
            if (!mPoints[i].mSide)
            {
                continue;
            }
            ImVec2 D = mPoints[i].mPosition - center;
            float distance = radius - sqrtf(D.x * D.x + D.y * D.y);

            mPoints[i].mPosition += (D * distance) * 0.1f;
        }
    }

    template<int count>void Subdivide(const int* index, std::map<uint32_t, int>& middles, int indexCenter) 
    {
        int halfSegmentIndex[count];
        for (int j = 0; j < count; j++)
        {
            int indexA = index[j];
            int indexB = index[(j + 1) % count];
            uint32_t key = (ImMin(indexA, indexB) << 16) + ImMax(indexA, indexB);
            auto iter = middles.find(key);
            if (iter == middles.end())
            {
                halfSegmentIndex[j] = (int)mPoints.size();
                bool isSide = mPoints[indexA].mSide && mPoints[indexB].mSide;
                mPoints.push_back({(mPoints[indexA].mPosition + mPoints[indexB].mPosition) * 0.5f, isSide });
                middles.insert(std::make_pair(key, halfSegmentIndex[j]));
            }
            else
            {
                halfSegmentIndex[j] = iter->second;
            }
        }
        for (int j = 0; j < count; j++)
        {
            int nextIndex = (j + 1) % count;
            mQuads.push_back({ indexCenter, halfSegmentIndex[j], index[nextIndex], halfSegmentIndex[nextIndex] });
        }
    }

    int GetAdjacentTriangles(int triIndex, int* adjacents)
    {
        int ref[] = {mTriangles[triIndex].mA, 
            mTriangles[triIndex].mB,
            mTriangles[triIndex].mC};

        int index = 0;
        for (int i = 0;i< mTriangles.size();i++)
        {
            if (i == triIndex || !mTriangles[i].mValid)
            {
                continue;
            }
            int local[] = { mTriangles[i].mA,
                mTriangles[i].mB,
                mTriangles[i].mC };

            int shareCount = 0;
            for(int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    if (ref[j] == local[k])
                    {
                        shareCount ++;
                        break;
                    }
                }
            }
            assert(shareCount < 3);
            if (shareCount == 2)
            {
                assert(index < 3);
                adjacents[index++] = i;
            }
        }
        return index;
    }

    ImVec2 worldToScreen(ImVec2 point)
    {
        ImGuiIO& io = ImGui::GetIO();
        float ratio = io.DisplaySize.x / io.DisplaySize.y;
        ImVec2 res = (point / (mSideSize * 3.5f));
        res.y *= ratio;
        res = res * io.DisplaySize;
        res += ImVec2(io.DisplaySize.x / 2.f, 0.f);
        return res;
    }

    int mSideSize = 0;
    int mBaseQuadCount = 0;
    int mSearchIterationCount = 0;
    struct Point
    {
        ImVec2 mPosition;
        bool mSide = false;
    };
    struct Triangle
    {
        int mA, mB, mC;
        bool mValid = true;
    };
    struct Quad
    {
        int mA, mB, mC, mD;
    };
    struct Neighbours
    {
        int mNeighbourCount = 0;
        int mNeighbour[MAX_NEIGHBOURS];
    };
    std::vector<Point> mPoints;
    std::vector<Triangle> mTriangles;
    std::vector<Quad> mQuads;
    std::vector<Neighbours> mNeighbours;
};

Hexagrid grid;

void DrawGrid(bool drawPositions, bool drawSector)
{
    ImDrawList* mDrawList;
    ImGuiIO& io = ImGui::GetIO();

    const ImU32 flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus;

#ifdef IMGUI_HAS_VIEWPORT
    ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos);
#else
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::SetNextWindowPos(ImVec2(0, 0));
#endif

    ImGui::PushStyleColor(ImGuiCol_WindowBg, 0);
    ImGui::PushStyleColor(ImGuiCol_Border, 0);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

    ImGui::Begin("debugDraw", NULL, flags);
    mDrawList = ImGui::GetWindowDrawList();
    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);

    const auto& points = grid.mPoints;
    if (drawPositions)
    {
        for (auto& point : grid.mPoints)
        {
            mDrawList->AddCircleFilled(grid.worldToScreen(point.mPosition), 5, point.mSide?0xFF303030:0xFF404040);
        }
    }
    if (drawSector)
    {
        for (auto& triangle : grid.mTriangles)
        {
            if (!triangle.mValid)
            {
                continue;
            }
            ImVec2 screenPoint[3] = {
                grid.worldToScreen(points[triangle.mA].mPosition),
                grid.worldToScreen(points[triangle.mB].mPosition),
                grid.worldToScreen(points[triangle.mC].mPosition),
            };
            mDrawList->AddPolyline(screenPoint, 3, 0xFFA0A0A0, true, 2.f);
        }
    
        for (int i = 0; i < grid.mBaseQuadCount; i++)
        {
            const auto& quad = grid.mQuads[i];
            ImVec2 screenPoint[4] = {
                grid.worldToScreen(points[quad.mA].mPosition),
                grid.worldToScreen(points[quad.mB].mPosition),
                grid.worldToScreen(points[quad.mC].mPosition),
                grid.worldToScreen(points[quad.mD].mPosition),
            };
            mDrawList->AddPolyline(screenPoint, 4, 0xFFFFFFFF, true, 2.f);
        }
    }
    else
    {
        for (int i = 0; i < grid.mPoints.size(); i++)
        {
            const auto& neighbour = grid.mNeighbours[i];
            ImVec2 ptA = grid.worldToScreen(points[i].mPosition);
            for (int j = 0; j < neighbour.mNeighbourCount; j++)
            {
                ImVec2 ptB = grid.worldToScreen(points[neighbour.mNeighbour[j]].mPosition);
                mDrawList->AddLine(ptA, ptB, 0xFFB0B0B0, 2.f);
            }
        }
    }
}

int main(int, char**)
{
    ImApp::ImApp imApp;

    ImApp::Config config;
    config.mWidth = 1280;
    config.mHeight = 720;
    //config.mFullscreen = true;
    imApp.Init(config);

    // Main loop
    while (!imApp.Done())
    {
        imApp.NewFrame();

        ImGuiIO& io = ImGui::GetIO();
        static bool firstFrame = true;
        static bool relaxIt = true;
        static bool relaxSide = false;
        static bool drawPositions = false;
        static bool drawSector = false;
        bool dirty = firstFrame;
        ImGui::Begin("Parameters");
        static int sideCount = 6;
        static int searchIterationCount = 6;
        static int seed = 1337;
        dirty |= ImGui::SliderInt("Sides", &sideCount, 2, 12);
        dirty |= ImGui::SliderInt("Seed", &seed, 1, 65535);
        dirty |= ImGui::SliderInt("Grouping", &searchIterationCount, 1, 20);
        dirty |= ImGui::Checkbox("Relax", &relaxIt);
        if (relaxIt)
        {
            dirty |= ImGui::Checkbox("Relax Side", &relaxSide);
        }

        ImGui::Checkbox("Draw Positions", &drawPositions);
        ImGui::Checkbox("Draw Sectors", &drawSector);

        ImGui::End();
        if (dirty)
        {
            grid.Init(sideCount, seed, searchIterationCount);
            firstFrame = false;
        }
        if (relaxIt)
        {
            grid.Relax();
        }
        if (relaxIt&&relaxSide)
        {
            grid.RelaxSide();
        }
        DrawGrid(drawPositions, drawSector);

        // render everything
        glClearColor(0.48f, 0.38f, 0.4f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render();

        imApp.EndFrame();
    }

    imApp.Finish();

    return 0;
}
