#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DInput/QInputAspect>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qpointlight.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/qfirstpersoncameracontroller.h>
#include <math.h>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_gamepad = NULL;

    view_dist = -600;
    view_angle_h = 0;
    view_angle_v = 45;
    joint[J1] = 0;
    joint[J2] = 120;//90;
    joint[J3] = -90;
    joint[J4] = 0;//90;
    joint[J5] = 0;

    memcpy(joint_old, joint, sizeof(joint));


    ui->btn_gamePad->setFocusPolicy(Qt::NoFocus);
    ui->btn_home->setFocusPolicy(Qt::NoFocus);

    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0xffffff)));
    view->installEventFilter(this);
    QWidget *container = QWidget::createWindowContainer(view);
    container->setFixedWidth(500);
    container->setFixedHeight(430);
    container->setParent(ui->view_widget);

    Qt3DInput::QInputAspect *input = new Qt3DInput::QInputAspect;
    view->registerAspect(input);

    // Root entity
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

    // Camera
    cameraEntity = view->camera();
    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 2000.0f);
    cameraEntity->setPosition(QVector3D(view_dist*sin(view_angle_h*M_PI/180),
                                        view_dist*cos(view_angle_h*M_PI/180),
                                        -view_dist*sin(view_angle_v*M_PI/180)));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 100));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(0, -1000, 1000.0f));
    lightEntity->addComponent(lightTransform);

    // For camera controls
    Qt3DExtras::QFirstPersonCameraController *camController = new Qt3DExtras::QFirstPersonCameraController(rootEntity);
    camController->setCamera(cameraEntity);

    scene = new Scene3D(rootEntity);
    view->setRootEntity(rootEntity);

    connect(this, SIGNAL(updateArmFK(int,float*,float*)), scene, SLOT(updateAngle(int,float*,float*)));
    connect(this, SIGNAL(updateArmIK(float*,float*,bool*)), scene, SLOT(updatePos(float*, float*,bool*)));
    connect(this, SIGNAL(updateAllJoint(float*, float*)), scene, SLOT(updateAll(float*, float*)));
    connect(this, SIGNAL(updateGripper(bool)), scene, SLOT(gripper(bool)));
    connect(this, SIGNAL(axixEn(bool)), scene, SLOT(axisShow(bool)));

    updateJointsVal();

    emit updateAllJoint(joint, pos);
    updatePosVal();

    memcpy(pos_old, pos, sizeof(pos));

    m_network = new Network(this);
    m_tcp = new TcpService();

    isAxisEn = false;

    t_system = new QTimer(this);
    connect(t_system, SIGNAL(timeout()), this, SLOT(systemState()));
    t_system->start(20);
    t_dirH = new QTimer(this);
    connect(t_dirH, SIGNAL(timeout()), this, SLOT(moveHori()));
    t_dirV = new QTimer(this);
    connect(t_dirV, SIGNAL(timeout()), this, SLOT(moveVert()));
    t_dirXY = new QTimer(this);
    connect(t_dirXY, SIGNAL(timeout()), this, SLOT(moveXY()));
    t_wristSwing = new QTimer(this);
    connect(t_wristSwing, SIGNAL(timeout()), this, SLOT(wristSwing()));
    t_wristRotate = new QTimer(this);
    connect(t_wristRotate, SIGNAL(timeout()), this, SLOT(wristRotate()));
    t_viewH = new QTimer(this);
    connect(t_viewH, SIGNAL(timeout()), this, SLOT(viewH()));
    t_viewV = new QTimer(this);
    connect(t_viewV, SIGNAL(timeout()), this, SLOT(viewV()));

    ui->btn_update->hide();

    isConnect   = false;
    isOpen = false;
    isHome = false;
    isUpdateJoint = true;
    isPlay = false;
    upPosCnt = 0;

//    POINT p;
//    p.x = pos[0];
//    p.y = pos[1];
//    p.z = pos[2];
//    vec_pos.push_back(p);
//    vec joint_vec(5);
//    joint_vec(J1) = joint[J1];
//    joint_vec(J2) = joint[J2];
//    joint_vec(J3) = joint[J3];
//    joint_vec(J4) = joint[J4];
//    joint_vec(J5) = joint[J5];
//    vec_joints.push_back(joint_vec);

    joint_v = 1.0;
    pos_v = 1.0;
    id_record = 0;
    QString txt = "Velocity: "+QString::number(1/0.02);
    scene->setText(txt);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    view_angle_h = value;
    cameraEntity->setPosition(QVector3D( view_dist*sin(view_angle_h*M_PI/180),
                                         view_dist*cos(view_angle_h*M_PI/180),
                                        -view_dist*cos(view_angle_v*M_PI/180)));
    cameraEntity->setUpVector(QVector3D(sin(view_angle_h*M_PI/180),
                                        cos(view_angle_h*M_PI/180),
                                        sin(view_angle_v*M_PI/180)));

    lightTransform->setTranslation(QVector3D( 1000*sin(view_angle_h*M_PI/180),
                                             -1000*cos(view_angle_h*M_PI/180),
                                              1000));
    ui->view_h->setText(QString::number(value));
    scene->textView(view_angle_h);
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    view_angle_v = value;
    cameraEntity->setPosition(QVector3D(view_dist*sin(view_angle_h*M_PI/180),
                                        view_dist*cos(view_angle_h*M_PI/180),
                                        -view_dist*cos(view_angle_v*M_PI/180)));
    cameraEntity->setUpVector(QVector3D(sin(view_angle_h*M_PI/180),
                                        cos(view_angle_h*M_PI/180),
                                        sin(view_angle_v*M_PI/180)));
    ui->view_v->setText(QString::number(value));
}

void MainWindow::on_slider_J1_valueChanged(int value)
{  
    float val = value;

    if (upPosCnt == 0)
    {
        emit updateArmFK(J1, &val, pos);
        updatePosVal();
    }

    if (isUpdateJoint)
    {
        joint[J1] = val;
        updateJointsVal();
    }
    sendCommand();
}

void MainWindow::on_slider_J2_valueChanged(int value)
{
    float val = value;

    if (upPosCnt == 0)
    {
        emit updateArmFK(J2, &val, pos);
        updatePosVal();
    }

    if (isUpdateJoint)
    {
        joint[J2] = val;
        updateJointsVal();
    }
    sendCommand();
}

void MainWindow::on_slider_J3_valueChanged(int value)
{
    float val = value;

    if (upPosCnt == 0)
    {
        emit updateArmFK(J3, &val, pos);
        updatePosVal();
    }

    if (isUpdateJoint)
    {
        joint[J3] = val;
        updateJointsVal();
    }
    sendCommand();
}

void MainWindow::on_slider_J4_valueChanged(int value)
{
    float val = value;

    if (upPosCnt == 0)
    {
        emit updateArmFK(J4, &val, pos);
        updatePosVal();
    }

    if (isUpdateJoint)
    {
        joint[J4] = val;
        updateJointsVal();
    }
    sendCommand();
}

void MainWindow::on_slider_J5_valueChanged(int value)
{
    float val = value;

    if (upPosCnt == 0)
    {
        emit updateArmFK(J5, &val, pos);
        updatePosVal();
    }

    if (isUpdateJoint)
    {
        joint[J5] = val;
        updateJointsVal();
    }
    sendCommand();
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
    static bool isMove = false;
    static int old_x = 0;
    static int diff = 0;

    switch(event->type())
    {
    case QEvent::MouseMove:
    {
        if (!isMove) break;
        int x = mouseEvent->x();
        if (old_x != 0)
        {

            if (abs(x - old_x) < 10)
                diff += (x - old_x);

        }
        old_x = x;
//        cameraEntity->setPosition(QVector3D( (view_dist+diff)*cos(view_angle_h*M_PI/180),
//                                             (view_dist+diff)*sin(view_angle_h*M_PI/180),
//                                            -view_dist*cos(view_angle_v*M_PI/180)));
        cameraEntity->setViewCenter(QVector3D(diff*cos(view_angle_h*M_PI/180),
                                              diff*sin(view_angle_h*M_PI/180),
                                              100));
    }
        break;
    case QEvent::MouseButtonPress:
        if (mouseEvent->button() == Qt::MiddleButton)
            isMove = true;
        break;
    case QEvent::MouseButtonRelease:
        if (mouseEvent->button() == Qt::MiddleButton)
            isMove = false;
        break;
    case QEvent::Wheel:
    {
        QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);

        QPoint delta = wheelEvent->angleDelta();

        if (delta.y() > 0)
            view_dist += 10;
        else
            view_dist -= 10;

        cameraEntity->setPosition(QVector3D( view_dist*sin(view_angle_h*M_PI/180),
                                             view_dist*cos(view_angle_h*M_PI/180),
                                            -view_dist*cos(view_angle_v*M_PI/180)));
        return true;
    }
        break;
    default:
        // standard event processing
        return QObject::eventFilter(obj, event);
        break;

    }
    return true;
}

void MainWindow::on_btn_joint6_clicked()
{

    isOpen = !isOpen;
    if (isOpen)
        ui->btn_joint6->setText("Close");
    else
        ui->btn_joint6->setText("Open");

    emit updateGripper(isOpen);
    sendCommand();
    updateJointsVal();
}

void MainWindow::on_btn_update_clicked()
{
    bool isValid;
    pos[0] = ui->val_x->text().toFloat();
    pos[1] = ui->val_y->text().toFloat();
    pos[2] = ui->val_z->text().toFloat();
    emit updateArmIK(pos, joint, &isValid);
    if (!isValid) {
        QMessageBox::critical(this,tr("Error"),tr("Invalid joint angle!"), QMessageBox::Ok);
        return;
    }

    emit updateAllJoint(joint, pos);
    sendCommand();
//    updateJointsVal();
    updatePosVal();
//    updateSlideVal();
    updateInverse();
}

void MainWindow::horiControl(double value)
{
    if (fabs(value) > 0.9)
    {
        hori_add = -value/fabs(value)*joint_v;
        t_dirH->start(20);

    }
    else if (fabs(value) < 0.3)
    {
        t_dirH->stop();
    }
}

void MainWindow::vertControl(double value)
{
    if (fabs(value) > 0.9)
    {
        vert_add = -value/fabs(value)*pos_v;
        t_dirV->start(20);
    }
    else if (fabs(value) < 0.3)
    {
        t_dirV->stop();

    }
}

void MainWindow::dirForwardBack(double value)
{
    if (fabs(value) > 0.8)
    {
        xy_add = value/fabs(value)*pos_v;
        t_dirXY->start(20);
    }
    else if (fabs(value) < 0.1)
    {
        t_dirXY->stop();   
    }

}

void MainWindow::speedUp(double value)
{
    if (fabs(value) > 0.8)
    {
        if (joint_v < 2)
            joint_v += 0.1;
        if (pos_v < 2)
            pos_v += 0.1;
    }

    QString txt = "Velocity: "+QString::number(joint_v/0.02);
    scene->setText(txt);
}

void MainWindow::speedDown(double value)
{
    if (fabs(value) > 0.8)
    {
        if (joint_v > 0.1)
            joint_v -= 0.1;
        if (pos_v > 0.1)
            pos_v -= 0.1;
    }

    QString txt = "Velocity: "+QString::number(joint_v/0.02);
    scene->setText(txt);
}

void MainWindow::swingRotate(double value)
{
    if (fabs(value) > 0.8)
    {
        swing_add = -value/fabs(value)*joint_v;
        t_wristSwing->start(20);
    }
    else
        t_wristSwing->stop();

}

void MainWindow::rotateCW(bool value)
{
    if (value)
    {
        rotate_add = -joint_v;
        t_wristRotate->start(20);
    }
    else {
        t_wristRotate->stop();
    }
}

void MainWindow::rotateCCW(bool value)
{
    if (value)
    {
        rotate_add = joint_v;
        t_wristRotate->start(20);
    }
    else {
        t_wristRotate->stop();
    }
}

void MainWindow::viewHChangeCCW(bool value)
{
    if (value)
    {
        viewH_add = 1;
        t_viewH->start(20);
    }
    else
        t_viewH->stop();
}

void MainWindow::viewHChangeCW(bool value)
{
    if (value)
    {
        viewH_add = -1;
        t_viewH->start(20);
    }
    else
        t_viewH->stop();
}

void MainWindow::viewVChangeUp(bool value)
{
    if (value)
    {
        viewV_add = 1;
        t_viewV->start(20);
    }
    else
        t_viewV->stop();
}

void MainWindow::viewVChangeDown(bool value)
{
    if (value)
    {
        viewV_add = -1;
        t_viewV->start(20);
    }
    else
        t_viewV->stop();
}

void MainWindow::systemState()
{
    float velocity = (joint[J1] - joint_old[J1])/0.1;
    ui->val_J1_v->setText(QString::number(velocity, 'f', 2));
    velocity = (joint[J2] - joint_old[J2])/0.1;
    ui->val_J2_v->setText(QString::number(velocity, 'f', 2));
    velocity = (joint[J3] - joint_old[J3])/0.1;
    ui->val_J3_v->setText(QString::number(velocity, 'f', 2));
    velocity = (joint[J4] - joint_old[J4])/0.1;
    ui->val_J4_v->setText(QString::number(velocity, 'f', 2));
    velocity = (joint[J5] - joint_old[J5])/0.1;
    ui->val_J5_v->setText(QString::number(velocity, 'f', 2));
    memcpy(joint_old, joint, sizeof(joint));

    velocity = (pos[X] - pos_old[X])/0.1;
    ui->val_x_v->setText(QString::number(velocity, 'f', 2));
    velocity = (pos[Y] - pos_old[Y])/0.1;
    ui->val_y_v->setText(QString::number(velocity, 'f', 2));
    velocity = (pos[Z] - pos_old[Z])/0.1;
    ui->val_z_v->setText(QString::number(velocity, 'f', 2));
    memcpy(pos_old, pos, sizeof(pos));

    if (isPlay && (id_record < vec_joints.size()))
    {
        uint8_t doneCnt = 0;
        float delta;

        for (int i = 0; i < 5; i++)
        {
            delta = joint_target[i] - joint[i];

            if (fabs(delta) < joint_v)
            {
                joint[i] = joint_target[i];
                doneCnt++;
            }
            else {

                joint[i] += delta/fabs(delta)*joint_v;
            }
        }

        emit updateAllJoint(joint, pos);
        sendCommand();
        updateJointsVal();
        updatePosVal();
        isUpdateJoint = false;
        updateSlideVal();
        isUpdateJoint = true;
        updateInverse();

        if (doneCnt == 5)
        {
            emit updateGripper(vec_joints[id_record](J6));
            id_record++;

            if (id_record < vec_joints.size())
            {
                joint_target[J1] = vec_joints[id_record](J1);
                joint_target[J2] = vec_joints[id_record](J2);
                joint_target[J3] = vec_joints[id_record](J3);
                joint_target[J4] = vec_joints[id_record](J4);
                joint_target[J5] = vec_joints[id_record](J5);
            }
            else {
                ui->btn_play->setIcon(QIcon("./resource/image/stop.png"));
                isPlay = false;
            }
        }

    }
}

void MainWindow::updateJointsVal()
{
    ui->val_J1->setText(QString::number(joint[J1], 'f', 2));
    ui->val_J2->setText(QString::number(joint[J2], 'f', 2));
    ui->val_J3->setText(QString::number(joint[J3], 'f', 2));
    ui->val_J4->setText(QString::number(joint[J4], 'f', 2));
    ui->val_J5->setText(QString::number(joint[J5], 'f', 2));
}

void MainWindow::updatePosVal()
{
    ui->val_x->setText(QString::number(pos[0], 'f', 2));
    ui->val_y->setText(QString::number(pos[1], 'f', 2));
    ui->val_z->setText(QString::number(pos[2], 'f', 2));
}

void MainWindow::updateSlideVal()
{
    ui->slider_J1->setValue((int)joint[J1]);
    ui->slider_J2->setValue((int)joint[J2]);
    ui->slider_J3->setValue((int)joint[J3]);
    ui->slider_J4->setValue((int)joint[J4]);
    ui->slider_J5->setValue((int)joint[J5]);
}

void MainWindow::updateInverse()
{
    ui->val_J1_i->setText(QString::number(joint[J1], 'f', 2));
    ui->val_J2_i->setText(QString::number(joint[J2], 'f', 2));
    ui->val_J3_i->setText(QString::number(joint[J3], 'f', 2));
    ui->val_J4_i->setText(QString::number(joint[J4], 'f', 2));
    ui->val_J5_i->setText(QString::number(joint[J5], 'f', 2));
}

void MainWindow::sendCommand()
{
    if (isConnect)
    {
        uint8_t len = 3+sizeof(float)*6;
        float j6 = isOpen;
        char buf[len];
        buf[0] = '@';
        buf[1] = '#';
        buf[2] = isHome ? 'a' : 'A';
        memcpy(&buf[3], joint, sizeof(float)*5);
        memcpy(&buf[3+sizeof(float)*5], &j6, sizeof(float));
//        m_network->sendCmd(buf, len);
        m_tcp->TcpSendMotor(buf, len);
    }
}

void MainWindow::wristSwing()
{
    float p_old[3];
    upPosCnt++;
    memcpy(p_old, pos, sizeof(pos));
    joint[J4] += swing_add;

    emit updateArmFK(J4, &joint[J4], pos);

    if (pos[2] < 28) {
        joint[J4] -= swing_add;
        memcpy(pos, p_old, sizeof(pos));
        upPosCnt--;
        return;
    }
    sendCommand();
    updatePosVal();
    updateJointsVal();
    isUpdateJoint = false;
    updateSlideVal();
    isUpdateJoint = true;
    upPosCnt--;
}

void MainWindow::wristRotate()
{
    joint[J5] += rotate_add;
    upPosCnt++;
    emit updateArmFK(J5, &joint[J5], pos);
    sendCommand();
    updatePosVal();
    updateJointsVal();
    isUpdateJoint = false;
    updateSlideVal();
    isUpdateJoint = true;
    upPosCnt--;
}

void MainWindow::moveXY()
{
    bool isValid;
    float norm = sqrt(pos[0]*pos[0] + pos[1]*pos[1]);
    int x_dir = cos(joint[J1]*D2R) >= 0 ? 1 : -1;
    int y_dir = sin(joint[J1]*D2R) >= 0 ? 1 : -1;
    float p_old[3];

    memcpy(p_old, pos, sizeof(pos));
    upPosCnt++;
    pos[0] += fabs(pos[0])/norm*xy_add*1*x_dir;
    pos[1] += fabs(pos[1])/norm*xy_add*1*y_dir;
    emit updateArmIK(pos, joint, &isValid);
    if (!isValid) {
        QMessageBox::critical(this,tr("Error"),tr("Invalid joint angle!"), QMessageBox::Ok);
        memcpy(pos, p_old, sizeof(pos));
        upPosCnt--;
        return;
    }

    emit updateAllJoint(joint, pos);
    sendCommand();
    updatePosVal();
    updateJointsVal();
    isUpdateJoint = false;
    updateSlideVal();
    isUpdateJoint = true;
    updateInverse();
    upPosCnt--;
}

void MainWindow::moveHori()
{
    joint[J1] += hori_add;
    emit updateArmFK(J1, &joint[J1], pos);
    ui->val_J1->setText(QString::number(joint[J1], 'f', 2));
    isUpdateJoint = false;
    ui->slider_J1->setValue((int(joint[J1])));
    isUpdateJoint = true;
    updatePosVal();
}

void MainWindow::moveVert()
{
    bool isValid;
    float z_old = pos[2];

    pos[2] = z_old + vert_add*1;
    upPosCnt++;
    emit updateArmIK(pos, joint, &isValid);
    if (!isValid) {
        QMessageBox::critical(this,tr("Error"),tr("Invalid joint angle!"), QMessageBox::Ok);
        pos[2] = z_old;
        upPosCnt--;
        return;
    }

    if (pos[2] < 28)
    {
        pos[2] = z_old;
        upPosCnt--;
        return;
    }
//    qDebug()<<"ret: "<<joint[J1]<<" "<<joint[J2]<<" "<<joint[J3]<<" "<<joint[J4]<<" "<<joint[J5];
    emit updateAllJoint(joint, pos);

    sendCommand();
    updateJointsVal();
    updatePosVal();
    isUpdateJoint = false;
    updateSlideVal();
    isUpdateJoint = true;
    updateInverse();
    upPosCnt--;
}

void MainWindow::viewH()
{
    view_angle_h += viewH_add;
    ui->horizontalSlider->setValue(view_angle_h);
}

void MainWindow::viewV()
{
    view_angle_v += viewV_add;
    ui->verticalSlider->setValue(view_angle_v);
}

void MainWindow::on_val_J1_returnPressed()
{
    float val = ui->val_J1->text().toFloat();
    emit updateArmFK(J1, &val, pos);
    joint[J1] = val;
    ui->slider_J1->setValue((int(joint[J1])));
    ui->val_J1->setText(QString::number(val));

}

void MainWindow::on_val_J2_returnPressed()
{
    float val = ui->val_J2->text().toFloat();
    emit updateArmFK(J2, &val, pos);
    joint[J2] = val;
    ui->slider_J2->setValue((int(joint[J2])));
    ui->val_J2->setText(QString::number(val));
    updatePosVal();
}

void MainWindow::on_val_J3_returnPressed()
{
    float val = ui->val_J3->text().toFloat();
    emit updateArmFK(J3, &val, pos);
    joint[J3] = val;
    ui->slider_J3->setValue((int(joint[J3])));
    ui->val_J3->setText(QString::number(val));
    updatePosVal();
}

void MainWindow::on_val_J4_returnPressed()
{
    float val = ui->val_J4->text().toFloat();
    emit updateArmFK(J4, &val, pos);
    joint[J4] = val;
    ui->slider_J4->setValue((int(joint[J4])));
    ui->val_J4->setText(QString::number(val));
    updatePosVal();
}

void MainWindow::on_val_J5_returnPressed()
{
    float val = ui->val_J5->text().toFloat();
    emit updateArmFK(J5, &val, pos);
    joint[J5] = val;
    ui->slider_J5->setValue((int(joint[J5])));
    ui->val_J5->setText(QString::number(val));
    updatePosVal();
}

void MainWindow::on_btn_gamePad_clicked()
{
    if (!m_gamepad) {
        auto gamepads = QGamepadManager::instance()->connectedGamepads();
        if (gamepads.isEmpty()) {
            qDebug()<<"can't find gamepad";
            m_gamepad = NULL;
        }
        else {
            ui->btn_gamePad->setIcon(QIcon("./resource/image/xbox-en.png"));

            m_gamepad = new QGamepad(*gamepads.begin(), this);
            connect(m_gamepad, SIGNAL(axisLeftXChanged(double)), this, SLOT(horiControl(double)));
            connect(m_gamepad, SIGNAL(axisLeftYChanged(double)), this, SLOT(vertControl(double)));
            connect(m_gamepad, SIGNAL(buttonBChanged(bool)), this, SLOT(gripperOpen(bool)));
            connect(m_gamepad, SIGNAL(buttonAChanged(bool)), this, SLOT(on_btn_record_clicked(bool)));
            connect(m_gamepad, SIGNAL(buttonGuideChanged(bool)), this, SLOT(home(bool)));
            connect(m_gamepad, SIGNAL(axisRightXChanged(double)), this, SLOT(dirForwardBack(double)));
            connect(m_gamepad, SIGNAL(axisRightYChanged(double)), this, SLOT(swingRotate(double)));
            connect(m_gamepad, SIGNAL(buttonL2Changed(double)), this, SLOT(speedDown(double)));
            connect(m_gamepad, SIGNAL(buttonR2Changed(double)), this, SLOT(speedUp(double)));
            connect(m_gamepad, SIGNAL(buttonR1Changed(bool)), this, SLOT(rotateCW(bool)));
            connect(m_gamepad, SIGNAL(buttonL1Changed(bool)), this, SLOT(rotateCCW(bool)));
            connect(m_gamepad, SIGNAL(buttonLeftChanged(bool)), this, SLOT(viewHChangeCCW(bool)));
            connect(m_gamepad, SIGNAL(buttonRightChanged(bool)), this, SLOT(viewHChangeCW(bool)));
            connect(m_gamepad, SIGNAL(buttonUpChanged(bool)), this, SLOT(viewVChangeUp(bool)));
            connect(m_gamepad, SIGNAL(buttonDownChanged(bool)), this, SLOT(viewVChangeDown(bool)));
            connect(m_gamepad, SIGNAL(buttonStartChanged(bool)), this, SLOT(on_btn_play_clicked(bool)));
        }
    }
}

void MainWindow::on_btn_axis_clicked()
{
    isAxisEn = !isAxisEn;
    if (isAxisEn)
        ui->btn_axis->setIcon(QIcon("./resource/image/axis-en.png"));
    else
        ui->btn_axis->setIcon(QIcon("./resource/image/axis-dis.png"));
    emit axixEn(isAxisEn);
}

void MainWindow::gripperOpen(bool val)
{
    if (val)
        on_btn_joint6_clicked();
}

void MainWindow::home(bool val)
{
    if (val)
    {
        isHome = true;
        on_btn_home_clicked(!val);
        isHome = false;
    }
}

void MainWindow::on_btn_connect_clicked()
{
    isConnect = !isConnect;
    if (isConnect)
    {
        ui->btn_connect->setIcon(QIcon("./resource/image/arm-online.png"));
    }
    else {
        ui->btn_connect->setIcon(QIcon("./resource/image/arm-offline.png"));
    }
}

void MainWindow::on_btn_home_clicked(bool checked)
{
    if (!checked)
    {
        isHome = true;
        joint[J1] = 0;
        joint[J2] = 120;//90;
        joint[J3] = -90;
        joint[J4] = 0;//90;
        joint[J5] = 0;

        emit updateAllJoint(joint, pos);
        sendCommand();
        updateJointsVal();
        updatePosVal();
        updateSlideVal();
        isHome = false;
    }
}

void MainWindow::on_btn_record_clicked(bool value)
{
    if (!value)
    {
        POINT p;
        vec joint_vec(6);

        joint_vec(J1) = joint[J1];
        joint_vec(J2) = joint[J2];
        joint_vec(J3) = joint[J3];
        joint_vec(J4) = joint[J4];
        joint_vec(J5) = joint[J5];
        joint_vec(J6) = isOpen;
        vec_joints.push_back(joint_vec);

        p.x = pos[0];
        p.y = pos[1];
        p.z = pos[2];
        vec_pos.push_back(p);

        for (int i = 0; i < vec_pos.size(); i++)
        {
    //        qDebug()<<vec_pos[i].x<<", "<<vec_pos[i].y<<", "<<vec_pos[i].z;
            qDebug()<<vec_joints[i](J1)<<", "<<vec_joints[i](J2)<<", "<<vec_joints[i](J3)<<", "
                    <<vec_joints[i](J4)<<", "<<vec_joints[i](J5);
        }

        scene->addPoint(vec_pos[vec_pos.size()-1]);
    }
}

void MainWindow::on_btn_play_clicked(bool value)
{

    if (!value)
    {
        isPlay = !isPlay;
        if (isPlay)
        {
            if (vec_joints.size() < 2)
            {
                isPlay = false;
                QMessageBox::warning(this,tr("Warning"),tr("There's not enough waypoint!"), QMessageBox::Ok);
            }
            else{
                qDebug()<<"playlist: "<<vec_joints.size();
                ui->btn_play->setIcon(QIcon("./resource/image/play.png"));
                id_record = 0;
                joint_target[J1] = vec_joints[id_record](J1);
                joint_target[J2] = vec_joints[id_record](J2);
                joint_target[J3] = vec_joints[id_record](J3);
                joint_target[J4] = vec_joints[id_record](J4);
                joint_target[J5] = vec_joints[id_record](J5);
                joint_target[J6] = vec_joints[id_record](J6);
            }
        }
        else {
            ui->btn_play->setIcon(QIcon("./resource/image/stop.png"));
        }
    }
}
