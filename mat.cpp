#include "mat.h"

/* ===================================
            MAT4 CLASS             
====================================*/

Mat4::Mat4(){
    this->externalTheta = false;
    //init matrix to zeros
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = 0.0f;
    
}

Mat4::Mat4(float** R, float* X){
    this->externalTheta = false;
    //init matrix to zeros
    for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) this->a[i][j] = R[i][j];
    for(int i = 0; i < 3; i++) this->a[3][i] = X[i];
    for(int i = 0; i < 3; i++) this->a[i][3] = 0.0f;
    this->a[3][3] = 1.0f;
}

Mat4::Mat4(float* theta, float alpha, float a, float d){
    this->externalTheta = true;
    this->alphaad[0] = alpha;
    this->alphaad[1] = a;
    this->alphaad[2] = d;
    this->update();
}

Mat4 Mat4::update(){
    if(not externalTheta) return *this;
    Mat4 TrZ;
    Mat4 RtZ;
    Mat4 RtX;
    Mat4 TrX;
    TrZ.generateRotZ(*(this->theta));
    TrZ.generateTrnZ(this->alphaad[0]);
    TrX.generateTrnX(this->alphaad[1]);
    RtX.generateRotX(this->alphaad[2]);
    Mat4 c = Mat4();
    c.write(TrZ.multiply(RtZ).multiply(RtX).multiply(TrX));
    this->write(c);
    return *this;
}

Mat4 Mat4::write(Mat4 b){
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = b.readCell(i, j);
    return *this;
}

Mat4 Mat4::multiply(Mat4 b){
    Mat4 c = Mat4();
    float s;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++){
            s = 0.0f;
            for(int l = 0; l < 4; l++) s += this->a[i][l] * b.readCell(l, j);
            c.writeCell(i, j, s);
        }
    this->write(c);
    return c;
}

Mat4 Mat4::generateIdentity(){
    //put 1s where i == j
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) 
        if(i == j) this->a[i][j] = 1.0f; 
        else this->a[i][j] = 0.0f;
    return *this;
}

Mat4 Mat4::generateTrnX(float val){
    this->generateIdentity();
    this->writeCell(0, 4, val);
    return *this;
}

Mat4 Mat4::generateTrnZ(float val){
    this->generateIdentity();
    this->writeCell(3, 4, val);
    return *this;
}

Mat4 Mat4::generateRotX(float theta){
    this->generateIdentity();
    this->writeCell(1, 1, cos(theta));
    this->writeCell(1, 2, sin(theta) * (-1));
    this->writeCell(2, 1, sin(theta));
    this->writeCell(2, 2, cos(theta));
    return *this;
}

Mat4 Mat4::generateRotZ(float theta){
    this->generateIdentity();
    this->writeCell(0, 0, cos(theta));
    this->writeCell(0, 1, sin(theta) * (-1));
    this->writeCell(1, 0, sin(theta));
    this->writeCell(1, 1, cos(theta));
    return *this;
}

Mat4 Mat4::multiply4DH(Mat4 TrZ, Mat4 RtZ, Mat4 RtX, Mat4 TrX){
    Mat4 c = Mat4();
    c.write(TrZ.multiply(RtZ).multiply(RtX).multiply(TrX));
    this->write(c);
    return c;
}

float Mat4::readCell(uint8_t row, uint8_t column){
    return a[row][column];
}

Mat4 Mat4::writeCell(uint8_t row, uint8_t column, float value){
    this->a[row][column] = value;
    return *this;
}

Mat4 Mat4::verbose(){
    printf(" \n ==== Printing Matrix === \n ");
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++)
            printf(" %2.2f ", this->a[i][j]);
        printf(" \n ");
        }
    printf(" \n ==== END OF PRINT === \n ");
    return *this;
}