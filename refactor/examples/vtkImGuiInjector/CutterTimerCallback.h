#pragma once

#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkObject.h>
#include <vtkPoints.h>
#include <vtkRenderWindowInteractor.h>


// 刀具路径动画的定时器回调
class CutterTimerCallback: public vtkCommand
{
public:
    static CutterTimerCallback* New()
    {
        CutterTimerCallback* cb = new CutterTimerCallback;
        cb->TimerCount = 0;
        return cb;
    }

    virtual void Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
    {
        if (vtkCommand::TimerEvent != eventId)
            return;

        vtkRenderWindowInteractor* iren = dynamic_cast<vtkRenderWindowInteractor*>(caller);
        if (!iren)
            return;

        // 如果定时器ID无效，直接返回
        if (this->TimerId <= 0)
            return;

        // 移动到下一个点
        ++this->CurrentIndex;

        // 检查是否到达终点或超出范围
        if (this->CurrentIndex >= this->NumPoints || this->TimerCount >= this->MaxSteps) {
            spdlog::info("Animation complete at point {}/{}", this->CurrentIndex, this->NumPoints);
            // 销毁定时器
            if (this->TimerId > 0) {
                spdlog::info("Destroying timer {} on completion", this->TimerId);
                iren->DestroyTimer(this->TimerId);
                this->TimerId = -1;  // 标记为已销毁
            }
            return;
        }

        // 更新刀具位置
        if (this->Points && this->Actor) {
            double position[3];
            this->Points->GetPoint(this->CurrentIndex, position);
            this->Actor->SetPosition(position);

            // 记录当前位置
            spdlog::info("Moving to point {}/{}: ({}, {}, {})",
                         this->CurrentIndex + 1,
                         this->NumPoints,
                         position[0],
                         position[1],
                         position[2]);

            // 重新渲染场景
            iren->GetRenderWindow()->Render();
        }

        ++this->TimerCount;
    }

    // 重置动画状态
    void Reset()
    {
        this->TimerCount = 0;
        this->CurrentIndex = this->StartIndex;
    }

    // 开始播放 - 不再需要设置Playing标志
    void Start()
    {
        // 开始播放不需要做任何事情，因为通过TimerId判断
    }

    // 停止播放 - 直接销毁定时器
    void Stop(vtkRenderWindowInteractor* iren)
    {
        if (this->TimerId > 0 && iren) {
            iren->DestroyTimer(this->TimerId);
            spdlog::info("Destroying timer {} on stop", this->TimerId);
            this->TimerId = -1;
        }
    }

    // 设置动画参数
    void SetActor(vtkActor* actor)
    {
        this->Actor = actor;
    }
    void SetPoints(vtkPoints* points)
    {
        this->Points = points;
        if (points) {
            this->NumPoints = points->GetNumberOfPoints();
        }
    }
    void SetStartIndex(int index)
    {
        this->StartIndex = index;
        this->CurrentIndex = index;
    }
    void SetMaxSteps(int steps)
    {
        this->MaxSteps = steps;
    }
    void SetTimerId(int id)
    {
        this->TimerId = id;
    }

public:
    int TimerCount = 0;           // 计时器计数
    int TimerId = 0;              // 计时器ID
    int MaxSteps = 1000;          // 最大步数
    int StartIndex = 0;           // 起始点索引
    int CurrentIndex = 0;         // 当前点索引
    int NumPoints = 0;            // 总点数
    vtkActor* Actor = nullptr;    // 要移动的Actor
    vtkPoints* Points = nullptr;  // 路径点集
};