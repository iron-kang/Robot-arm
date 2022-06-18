#include "scene3d.h"
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QGeometry>

Scene3D::Scene3D(Qt3DCore::QEntity *rootEntity)
    : m_rootEntity(rootEntity)
{
    arm = new RobotArm(m_rootEntity);

#if 0
    // Plane shape data
    Qt3DExtras::QPlaneMesh *planeMesh = new Qt3DExtras::QPlaneMesh();
    planeMesh->setWidth(2);
    planeMesh->setHeight(2);

    // Plane mesh transform
    Qt3DCore::QTransform *planeTransform = new Qt3DCore::QTransform();
    planeTransform->setScale(100.0f);
    planeTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 90.0f));

    Qt3DExtras::QPhongMaterial *planeMaterial = new Qt3DExtras::QPhongMaterial();
    planeMaterial->setDiffuse(QColor(QRgb(0xa69929)));

    // Plane
    Qt3DCore::QEntity *m_planeEntity = new Qt3DCore::QEntity(m_rootEntity);
    m_planeEntity->addComponent(planeMesh);
    m_planeEntity->addComponent(planeMaterial);
    m_planeEntity->addComponent(planeTransform);
#endif
    // World Z-axis
    cylinderZ = new Qt3DExtras::QCylinderMesh();
    cylinderZ->setRadius(1.5);
    cylinderZ->setLength(300);
    cylinderZ->setRings(100);
    cylinderZ->setSlices(20);
    Qt3DCore::QTransform *cylinderZTransform = new Qt3DCore::QTransform();
    cylinderZTransform->setRotationX(90);
    cylinderZTransform->setTranslation(QVector3D(0.0f, 0.0f, 200));
    Qt3DExtras::QPhongMaterial *cylinderZMaterial = new Qt3DExtras::QPhongMaterial();
    cylinderZMaterial->setDiffuse(QColor(QRgb(0x0000ff)));
    m_cylinderZEntity = new Qt3DCore::QEntity(m_rootEntity);
    m_cylinderZEntity->addComponent(cylinderZ);
    m_cylinderZEntity->addComponent(cylinderZMaterial);
    m_cylinderZEntity->addComponent(cylinderZTransform);
    m_cylinderZEntity->removeComponent(cylinderZ);

    // World X-axis
    cylinderX = new Qt3DExtras::QCylinderMesh();
    cylinderX->setRadius(1.5);
    cylinderX->setLength(300);
    cylinderX->setRings(100);
    cylinderX->setSlices(20);
    Qt3DCore::QTransform *cylinderXTransform = new Qt3DCore::QTransform();
    cylinderXTransform->setRotationZ(90);
    cylinderXTransform->setTranslation(QVector3D(200, 0.0f, 0));
    Qt3DExtras::QPhongMaterial *cylinderXMaterial = new Qt3DExtras::QPhongMaterial();
    cylinderXMaterial->setDiffuse(QColor(QRgb(0xff0000)));
    m_cylinderXEntity = new Qt3DCore::QEntity(m_rootEntity);
    m_cylinderXEntity->addComponent(cylinderX);
    m_cylinderXEntity->addComponent(cylinderXMaterial);
    m_cylinderXEntity->addComponent(cylinderXTransform);
    m_cylinderXEntity->removeComponent(cylinderX);

    // World Y-axis
    cylinderY = new Qt3DExtras::QCylinderMesh();
    cylinderY->setRadius(1.5);
    cylinderY->setLength(300);
    cylinderY->setRings(100);
    cylinderY->setSlices(20);
    Qt3DCore::QTransform *cylinderYTransform = new Qt3DCore::QTransform();
    cylinderYTransform->setTranslation(QVector3D(0, 200, 0));
    Qt3DExtras::QPhongMaterial *cylinderYMaterial = new Qt3DExtras::QPhongMaterial();
    cylinderYMaterial->setDiffuse(QColor(QRgb(0x00ff00)));
    m_cylinderYEntity = new Qt3DCore::QEntity(m_rootEntity);
    m_cylinderYEntity->addComponent(cylinderY);
    m_cylinderYEntity->addComponent(cylinderYMaterial);
    m_cylinderYEntity->addComponent(cylinderYTransform);
    m_cylinderYEntity->removeComponent(cylinderY);

    auto *root = new Qt3DCore::QEntity(m_rootEntity);
    for (float i = -8; i <= 8; i++)
    {
        drawLine({ -400, i*50, 0 }, {  400, i*50, 0 }, Qt::gray, root);
        drawLine({ i*50, -400, 0 }, { i*50,  400, 0 }, Qt::gray, root);
    }

    textTF = new Qt3DCore::QTransform();
    textTF->setTranslation(QVector3D(150, 0, 250));
    textTF->setScale(1.5);
    textTF->setRotationX(90);
    text2d = new Qt3DExtras::QText2DEntity(m_rootEntity);
    text2d->setText("Test!!!");
    text2d->setHeight(50);
    text2d->setWidth(100);
    text2d->setColor(Qt::gray);
    text2d->addComponent(textTF);

    sphereMesh = new Qt3DExtras::QSphereMesh();
    sphereMesh->setRings(20);
    sphereMesh->setSlices(20);
    sphereMesh->setRadius(5);
    sphereMaterial = new Qt3DExtras::QPhongMaterial();
    sphereMaterial->setDiffuse(QColor(QRgb(0xff0000)));

    angle = 0;

}

Scene3D::~Scene3D()
{

}

void Scene3D::textView(int value)
{
    QMatrix4x4 matrix;
    matrix = textTF->matrix();
    matrix.setToIdentity();
    matrix.rotate(90, QVector3D(cos(-value*M_PI/180), sin(-value*M_PI/180), 0.0f));
    matrix.rotate(-value, QVector3D(0.0f, 0.0f, 1.0f));
    textTF->setMatrix(matrix);
    textTF->setScale(1.5);
    textTF->setTranslation(QVector3D(150*cos(-value*M_PI/180), 150*sin(-value*M_PI/180), 250));
}

void Scene3D::setText(QString str)
{
    text2d->setText(str);
}

void Scene3D::updateAngle(int joint, float *angle, float *pos)
{
    arm->setJoint(joint, angle);
    arm->updateFK(pos);
//    qDebug()<<"FK x: "<<pos[0]<<", y: "<<pos[1]<<", z: "<<pos[2]<<" ("<<joint<<")";
}

void Scene3D::gripper(bool value)
{
    arm->gripper(value);
}

void Scene3D::updateAll(float *joints, float *pos)
{
//    qDebug()<<"FK "<<joints[J1]<<" "<<joints[J2]<<" "<<joints[J3]<<" "<<joints[J4]<<" "<<joints[J5];
    arm->setJoints(joints);
    arm->updateFK(pos);
}

void Scene3D::updatePos(float *pos, float *joint, bool *valid)
{
//    qDebug()<<"IK x: "<<pos[0]<<", y: "<<pos[1]<<", z: "<<pos[2];
    arm->setPos(pos);
    *valid = arm->updateIK(joint);
}

void Scene3D::drawLine(const QVector3D& start, const QVector3D& end, const QColor& color, Qt3DCore::QEntity *_rootEntity)
{
    auto *geometry = new Qt3DRender::QGeometry(_rootEntity);

    // position vertices (start and end)
    QByteArray bufferBytes;
    bufferBytes.resize(3 * 2 * sizeof(float)); // start.x, start.y, start.end + end.x, end.y, end.z
    float *positions = reinterpret_cast<float*>(bufferBytes.data());
    *positions++ = start.x();
    *positions++ = start.y();
    *positions++ = start.z();
    *positions++ = end.x();
    *positions++ = end.y();
    *positions++ = end.z();

    auto *buf = new Qt3DRender::QBuffer();
    buf->setData(bufferBytes);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(2);
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry

    // connectivity between vertices
    QByteArray indexBytes;
    indexBytes.resize(2 * sizeof(unsigned int)); // start to end
    unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
    *indices++ = 0;
    *indices++ = 1;

    auto *indexBuffer = new Qt3DRender::QBuffer();
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(2);
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometry

    // mesh
    auto *line = new Qt3DRender::QGeometryRenderer(_rootEntity);
    line->setGeometry(geometry);
    line->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    auto *material = new Qt3DExtras::QPhongMaterial(_rootEntity);
    material->setAmbient(color);

    // entity
    auto *lineEntity = new Qt3DCore::QEntity(_rootEntity);
    lineEntity->addComponent(line);
    lineEntity->addComponent(material);
}

void Scene3D::axisShow(bool en)
{
    if (en)
    {
        m_cylinderZEntity->addComponent(cylinderZ);
        m_cylinderYEntity->addComponent(cylinderY);
        m_cylinderXEntity->addComponent(cylinderX);
    }
    else
    {
        m_cylinderZEntity->removeComponent(cylinderZ);
        m_cylinderYEntity->removeComponent(cylinderY);
        m_cylinderXEntity->removeComponent(cylinderX);
    }
    arm->axisShow(en);
}

void Scene3D::addPoint(POINT p)
{

    Qt3DCore::QTransform* m_position_TF = new Qt3DCore::QTransform();
    m_position_TF->setTranslation(QVector3D(p.x, p.y, p.z));

    Qt3DCore::QEntity *m_position = new Qt3DCore::QEntity(m_rootEntity);
    m_position->addComponent(sphereMesh);
    m_position->addComponent(sphereMaterial);
    m_position->addComponent(m_position_TF);
}
