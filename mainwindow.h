#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "scene3d.h"
#include <Qt3DRender/qcamera.h>
#include <QMouseEvent>
#include <QtGamepad/QGamepad>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <network.h>
#include <vector>
#include "tcpservice.h"

namespace Ui {
class MainWindow;
}

using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_horizontalSlider_valueChanged(int value);
    void on_verticalSlider_valueChanged(int value);
    void on_slider_J1_valueChanged(int value);
    void on_slider_J2_valueChanged(int value);
    void on_slider_J3_valueChanged(int value);
    void on_slider_J4_valueChanged(int value);
    void on_slider_J5_valueChanged(int value);
    void on_btn_joint6_clicked();
    void on_btn_update_clicked();
    void horiControl(double value);
    void vertControl(double value);
    void swingRotate(double value);
    void speedUp(double value);
    void speedDown(double value);
    void rotateCW(bool value);
    void rotateCCW(bool value);
    void viewHChangeCCW(bool value);
    void viewHChangeCW(bool value);
    void viewVChangeUp(bool value);
    void viewVChangeDown(bool value);
    void gripperOpen(bool val);
    void home(bool val);
    void moveHori();
    void moveVert();
    void moveXY();
    void wristSwing();
    void wristRotate();
    void viewH();
    void viewV();
    void systemState();
    void dirForwardBack(double val);
    void on_val_J1_returnPressed();
    void on_val_J2_returnPressed();
    void on_val_J3_returnPressed();
    void on_val_J4_returnPressed();
    void on_val_J5_returnPressed();
    void on_btn_gamePad_clicked();
    void on_btn_axis_clicked();
    void on_btn_connect_clicked();
    void on_btn_home_clicked(bool checked);
    void on_btn_record_clicked(bool value);
    void on_btn_play_clicked(bool value);

private:
    Ui::MainWindow *ui;
    Qt3DRender::QCamera *cameraEntity ;
    Qt3DCore::QTransform *lightTransform;
    QGamepad *m_gamepad;
    QTimer *t_dirH, *t_dirV, *t_dirXY;
    QTimer *t_wristSwing, *t_wristRotate;
    QTimer *t_viewH, *t_viewV;
    QTimer *t_system;
    QPushButton *btn_gamePad;
    Network *m_network;
    TcpService *m_tcp;
    Scene3D *scene ;

    vector<POINT> vec_pos;
    vector<vec> vec_joints;
    int view_dist;
    int view_angle_h, view_angle_v;
    int upPosCnt;
    unsigned int id_record;
    float pos[3], pos_old[3];
    float joint[6], joint_old[6], joint_target[6];
    float hori_add, vert_add, xy_add, swing_add, rotate_add;
    float viewH_add, viewV_add;
    float joint_v, pos_v;
    bool isAxisEn;
    bool isConnect;
    bool isOpen = false;
    bool isHome;
    bool isUpdateJoint;
    bool isPlay;

    bool eventFilter(QObject *obj, QEvent *event);
    void updatePosVal();
    void updateJointsVal();
    void updateSlideVal();
    void updateInverse();
    void sendCommand();

signals:
    void updateArmFK(int joint, float *angle, float *pos);
    void updateArmIK(float *pos, float *joint, bool *valid);
    void updateAllJoint(float *joints, float *pos);
    void updateGripper(bool value);
    void axixEn(bool en);

};

#endif // MAINWINDOW_H
