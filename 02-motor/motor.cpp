#include <iostream>
#include <windows.h>
using namespace std;
int speed = 0, direct = 0;
class MotorController
{
public:
    bool open = 0;
    MotorController() : speed(0), direction(0) {}

    void setSpeed(int newSpeed)
    {
        speed = newSpeed;
    }

    int getSpeed() const
    {
        return speed;
    }

    void setDirection(int newDirection)
    {
        direction = newDirection;
    }

    int getDirection() const
    {
        return direction;
    }

private:
    int speed;
    int direction;
};

MotorController motor;
void show();
void work();

int main()
{
    motor.setSpeed(speed);
    motor.setDirection(direct);
    while (1)
    {
        show();
        work();
        system("cls");
    }
}

void show()
{
    cout << "电机状态：";
    if (motor.open)
    {
        cout << "开机" << endl;
        cout << "电机转速(r/min)：" << motor.getSpeed() << endl;
        cout << "电机方向：";
        if (motor.getDirection() == 1)
            cout << "逆时针" << endl;
        else
            cout << "顺时针" << endl;
    }
    else
    {
        cout << "关机" << endl;
    }
}
void work()
{
    if (motor.open)
    {
        cout << "-------------------------------" << endl;
        cout << "请输入操作：" << endl;
        cout << "1.关机" << endl;
        cout << "2.设置速度" << endl;
        cout << "3.设置方向" << endl;
        cout << "4.退出" << endl;
        int choice;
        cin >> choice;
        switch (choice)
        {
        case 1:
            motor.open = 0;
            break;
        case 2:

            cout << "请输入速度(r/min):" << endl;
            cin >> speed;
            if (speed < 0)
            {
                cout << "输入正数,哥们" << endl;
                Sleep(1000);
                break;
            }
            else if (speed > 20000)
            {
                cout << "你家电机能转这么快啊,哥们" << endl;
                Sleep(1000);
                break;
            }
            motor.setSpeed(speed);
            break;
        case 3:

            cout << "请输入方向(0顺/1逆):" << endl;
            cin >> direct;
            if (direct != 1 && direct != 0)
            {
                cout << "输入有误,哥们,请输入0或1" << endl;
                Sleep(2000);
                break;
            }
            motor.setDirection(direct);
            break;
        case 4:
            exit(0);
            break;
        case 5:

        default:
            cout << "输入有误,哥们,请重新输入" << endl;
            Sleep(2000);
            break;
        }
    }
    else
    {
        cout << "输入1开机" << endl;
        int choice;
        cin >> choice;
        if (choice == 1)
        {
            motor.open = 1;
            motor.setSpeed(0);
            motor.setDirection(0);
        }
        else
        {
            cout << "输入有误,哥们,请重新输入" << endl;
            Sleep(1000);
            system("cls");
        }
    }
}