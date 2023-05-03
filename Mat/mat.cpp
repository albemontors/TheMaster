#include "mat.h"

/* ===================================
            MAT4 CLASS             
====================================*/

Mat4::Mat4(){
    //init matrix to zeros
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = 0.0f;
    
}

Mat4::Mat4(float R[3][3], float* X){
    //init matrix to zeros
    for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) this->a[i][j] = R[i][j];
    for(int i = 0; i < 3; i++) this->a[3][i] = X[i];
    for(int i = 0; i < 3; i++) this->a[i][3] = 0.0f;
    this->a[3][3] = 1.0f;
}

Mat4::Mat4(float alpha, float a, float d){
    this->alphaad[0] = alpha;
    this->alphaad[1] = a;
    this->alphaad[2] = d;
    this->update(0.0f);
}

Mat4 Mat4::setParam(float alpha, float a, float d){
    this->alphaad[0] = alpha;
    this->alphaad[1] = a;
    this->alphaad[2] = d;
    this->update(0.0f);
    return *this;
}

Mat4 Mat4::update(float _theta){
    Mat4 TrZ;
    Mat4 RtZ;
    Mat4 RtX;
    Mat4 TrX;
    RtZ.generateRotZ(_theta);
    TrZ.generateTrnZ(this->alphaad[2]);
    TrX.generateTrnX(this->alphaad[1]);
    RtX.generateRotX(this->alphaad[0]);
    this->multiply4DH(TrZ, RtZ, RtX, TrX);
    return *this;
}

Mat4 Mat4::write(Mat4 b){
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = b.readCell(i, j);
    return *this;
}

Mat4 Mat4::writeAll(Mat4 b){
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = b.readCell(i, j);
    for(int i = 0; i < 3; i++) this->alphaad[i] = b.alphaad[i];
    return *this;
}

Mat4 Mat4::multiply(Mat4 b){
    Mat4 c = Mat4();
    float s;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++){
            s = 0.0f;
            for(int l = 0; l < 4; l++) s += this->a[l][j] * b.readCell(i, l);
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
    this->writeCell(0, 3, val);
    return *this;
}

Mat4 Mat4::generateTrnZ(float val){
    this->generateIdentity();
    this->writeCell(2, 3, val);
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
    this->write(RtX.multiply(TrX).multiply(TrZ).multiply(RtZ));
    return *this;
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