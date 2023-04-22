#ifndef _MATH_H
#define _MATH_H

#include "definitions.h"

#include <math.h>

class Mat4 {
    public:
        /**
        * @brief Constructor, makes a 0s 4x4 matrix
        * @return this
        */
        Mat4();
        /**
        * @brief Constructor, makes a 4x4 matrix starting from a square array
        * @param R[3][3] rotational matrix for the transformation
        * @param X[3] vector for x,y,z
        * @return this
        */
        Mat4(float** R, float* X);
        /**
        * @brief Constructor, makes a 4x4 matrix starting from a square array
        * @param theta to be a pointer to the value of theta
        * @param alpha hartenbergs value alpha
        * @param a hartenbergs value a
        * @param d hartenbergs value d
        * @return this
        */
        Mat4(float* theta, float alpha, float a, float d);
        /**
        * @brief updates the theta value for matrices defined from pointed value
        * @return this
        */
        Mat4 update();
        /**
        * @brief changes the pointed value for Theta
        * @return this
        */
        Mat4 assignTheta(float**);
        /**
        * @brief writes the input into this
        * @param matrix to write into this
        * @return this
        */
        Mat4 write(Mat4 b);    
        /**
        * @brief row by column multiplication between this and param
        * @param b second member of multiplication
        * @return this
        */                              
        Mat4 multiply(Mat4 b);
        /**
        * @brief writes this with identity 4x4 matrix
        * @return this
        */
        Mat4 generateIdentity(); 
        /**
         * @brief Writes the translation matrix around X axis on this
         * @param val lenght of translation
         * @return this
         */                          
        Mat4 generateTrnX(float val);
        /**
         * @brief Writes the translation matrix around Z axis on this
         * @param val lenght of translation
         * @return this
         */                                            
        Mat4 generateTrnZ(float val); 
        /**
         * @brief Writes the rotational matrix around X axis on this
         * @param val angle in rads
         * @return this
         */                                                  
        Mat4 generateRotX(float theta);
        /**
         * @brief Writes the rotational matrix around Z axis on this
         * @param val angle in rads
         * @return this
         */                                           
        Mat4 generateRotZ(float theta);
        /**
         * @brief multiplyes the 4 matrices in row and writes on this
         * @param RtZ matrix of rotation around Z
         * @param TrZ matrix of translation along Z
         * @param TrX matrix of translation along X
         * @param RtX matrix of rotation around X
         * @return this
         */                                          
        Mat4 multiply4DH(Mat4 RtZ, Mat4 TrZ, Mat4 TrX, Mat4 RtX);
        /**
        * @brief reads a cell
        * @param row number [0-3]
        * @param column number [0-3]
        * @return this
        */
        float readCell(uint8_t row, uint8_t column);
        /**
        * @brief writes a cell
        * @param row number [0-3]
        * @param column number [0-3]
        * @param value to write
        * @return cell content
        */
        Mat4 writeCell(uint8_t row, uint8_t column, float value);
        /**
        * @brief prints the matrix to terminal in a fancy layout
        * @return this
        */
        Mat4 verbose();

    private:
        float a[4][4];
        bool externalTheta;
        float* theta;
        float alphaad[3];
};

#endif