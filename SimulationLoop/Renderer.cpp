#include "Renderer.h"
#include "Math\Matrix4.h"

GLuint Renderer::m_sphereTex;

bool Renderer::Init()
{
	m_sphereTex = TextureLoader::LoadBMP("checker.bmp");


    return true;
}

void Renderer::DrawRect(const Rectangle2D& r, math::Vector3D c)
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(c.x, c.y, c.z);
    glRectf(r.origin.x, r.origin.y, r.origin.x + r.size.x, r.origin.y + r.size.y);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::DrawSphere(const Sphere& s)
{
    //glEnable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
	glTranslatef(s.position.x, s.position.y, 0);
    glColor3f(s.color.x, s.color.y, s.color.z);

	//glBindTexture(GL_TEXTURE_2D, m_sphereTex);
	GLUquadric* quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_FILL);
	gluQuadricTexture(quadric, GL_TRUE);
	gluQuadricNormals(quadric, GLU_SMOOTH);
	gluSphere(quadric, s.radius, 20, 20);
	glPopMatrix();
}

void Renderer::DrawAABBCube(const AABB& c)
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    //glScalef(c.size.x, c.size.y, c.size.z);
    //glTranslatef(c.position.x, c.position.y, c.position.z);

    math::Matrix4 scale = math::scale(c.size);
    math::Matrix4 translation = math::translation(c.position);

    math::Matrix4 transform = scale * translation;

    glMultMatrixf(transform.data());

    glColor3f(c.color.x, c.color.y, c.color.z);

    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    //glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);

    // Bottom face (y = -1.0f)
    //glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);

    // Front face  (z = 1.0f)
    //glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);

    // Back face (z = -1.0f)
    //glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    glVertex3f(1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);

    // Left face (x = -1.0f)
    //glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);

    // Right face (x = 1.0f)
    //glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);
    glEnd();  // End of drawing color-cube

    glPopMatrix();
}

void Renderer::DrawOBBCube(const OBB& c)
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    
    math::Matrix4 scale = math::scale(c.size);
    math::Matrix4 rotation = math::fromMatrix3(c.orientation);
    math::Matrix4 translation = math::translation(c.position);

    math::Matrix4 transform = scale * rotation * translation;

    //glTranslatef(c.position.x, c.position.y, c.position.z);
    //glRotatef(45.f, 0, 1, 1);    
    //glScalef(c.size.x, c.size.y, c.size.z);

    glMultMatrixf(transform.data());

    glColor3f(c.color.x, c.color.y, c.color.z);

    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    //glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);

    // Bottom face (y = -1.0f)
    //glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);

    // Front face  (z = 1.0f)
    //glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);

    // Back face (z = -1.0f)
    //glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    glVertex3f(1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);

    // Left face (x = -1.0f)
    //glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);

    // Right face (x = 1.0f)
    //glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);
    glEnd();  // End of drawing color-cube

    glPopMatrix();
}
