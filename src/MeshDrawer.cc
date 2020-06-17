#include <MeshDrawer.h>
#include <pangolin/pangolin.h>
#include <opencv2/highgui/highgui.hpp>

namespace defSLAM
{
void MeshDrawer::addTextureImage(cv::Mat im){
    Texture = im.clone();
}

void MeshDrawer::AddEdge(std::vector<int> edge){
    for (uint i(0);i<2;i++){
        this->Edges.push_back(edge[i]);
    }
}

void MeshDrawer::AddFacet(std::vector<int> fac){
    for (uint i(0);i<3;i++){
        this->Facets.push_back(fac[i]);
    }
}

void MeshDrawer::AddNode(std::vector<double> pos,std::vector<double> proj,int role){
    for (uint i(0);i<3;i++){
        this->Nodes.push_back(pos[i]);
    }
    for (uint i(0);i<2;i++){    
        this->NodesProjection.push_back(proj[i]);
    }
    this->NodesRole.push_back(role);
}

void MeshDrawer::DrawMesh(double alpha,bool drawedges){
    std::unique_lock<std::mutex> M(this->mtemp);
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable(GL_TEXTURE_2D);
    cv::Mat fin_image;
    std::vector<cv::Mat> channels;
    channels.resize(3);
    cv::split(this->Texture,channels);

    cv::Mat AlphaChannel(this->Texture.rows,this->Texture.cols,CV_8UC1,cv::Scalar(alpha*255));

    channels.push_back(AlphaChannel);
    cv::merge(channels,fin_image);

    GLuint texID;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_2D, texID);
    glPixelStoref(GL_UNPACK_ALIGNMENT, 1);
    glLineWidth(1);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, fin_image.cols, fin_image.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, fin_image.ptr());

    for(uint i = 0;i < Facets.size();i = i + 3){
        glBegin(GL_TRIANGLES);
        glColor3f(1,1,1); // set global color to white, otherwise this color will be (somehow) added to the texture
        for(uint j=i; j<i+3; j++){
            glTexCoord2f(this->NodesProjection[2*Facets[j]],this->NodesProjection[2*Facets[j]+1]);
            glVertex3f(this->Nodes[3*Facets[j]],this->Nodes[3*Facets[j]+1],this->Nodes[3*Facets[j]+2]);
        }
    }

    glEnd();
    glDeleteTextures(1, &texID); //TODO load all textures once and only remove when it change


    glDisable(GL_TEXTURE_2D);

    if(drawedges){
        // Draw Edges
    for(uint i = 0;i < this->Edges.size();i = i + 2){
        glBegin(GL_LINES);

        for(uint j=i; j<i+2; j++){
            double Vx = this->Nodes[3*Edges[j]];
            double Vy = this->Nodes[3*Edges[j]+1];
            double Vz = this->Nodes[3*Edges[j]+2];
            int role = this->NodesRole[Edges[j]];
            if (role == 0){
                glColor3f(0.1,0,1);
                glVertex3f(Vx,Vy,Vz);
            }else if (role == 1){
                glColor3f(0.1,0.9,0.1);
                glVertex3f(Vx,Vy,Vz);
            }else if (role == 2){
                glColor3f(1.0,0.8,0.1);
                glVertex3f(Vx,Vy,Vz);
            }else if (role == 3){
                glColor3f(0.9,0.1,0.1);
                glVertex3f(Vx,Vy,Vz);
            }else{
                glColor3f(0.05,0.05,0.05);
                glVertex3f(Vx,Vy,Vz);
            }
        }
    }
            glEnd();
    }
    }
}

