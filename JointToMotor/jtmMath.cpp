#include "jointToMotor.h"

bool invert(float** M) {
    
    float A[AXIS_COUNT][AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) A[i][j] = M[i][j];

    float det = getMatrixDeterminant((float**)M);
    if (det == 0) return true;

    for (int i = 0; i < AXIS_COUNT; i++)
    {
        for (int j = 0; j < AXIS_COUNT; j++)
        {
            A[j][i] = getComplementOf((float**)M, j, i) / det;
        }
    }

    transponseMatrix((float**)A);

    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) M[i][j] = A[i][j];

    return false;
}

float getMatrixDeterminant(float** M)
{
    float determinante = 0;
    if (AXIS_COUNT == 1)
    {
        determinante = M[0][0];
    }
    if (AXIS_COUNT == 2)
    {
        determinante = M[0][0] * M[1][1] - M[1][0] * M[0][1];
    }
    else
    {
        for (int i = 0; i < AXIS_COUNT; i++)
        {
            float A[AXIS_COUNT - 1][AXIS_COUNT - 1];
            int cy = 0;
            for (int y = 1; y < AXIS_COUNT; y++)
            {
                int cx = 0;
                for (int x = 0; x < AXIS_COUNT; x++)
                {
                    if (x != i)
                    {
                        A[cx][cy] = M[x][y];
                        cx++;
                    }
                }
                cy++;
            }

            determinante += M[i][0] * pow(-1, i + 0) * getMatrixDeterminant((float**)A);
        }
    }

    return determinante;
}

float getComplementOf(float** M, int X, int Y) {
    float det;

    if (AXIS_COUNT == 2)
    {
        det = M[1 - X][1 - Y];
    }
    else
    {
        float A[AXIS_COUNT - 1][AXIS_COUNT - 1];
        int cy = 0;
        for (int y = 0; y < AXIS_COUNT; y++)
        {
            if (y != Y)
            {
                int cx = 0;
                for (int x = 0; x < AXIS_COUNT; x++)
                {
                    if (x != X)
                    {
                        A[cx][cy] = M[x][y];
                        cx++;
                    }
                }
                cy++;
            }
        }
        det = getMatrixDeterminant((float**)A);
    }

    return (pow(-1, X + Y) * det);
}

void transponseMatrix(float** M) {
    float A[AXIS_COUNT][AXIS_COUNT];

    for (int i = 0; i < AXIS_COUNT; i++)
    {
        for (int j = 0; j < AXIS_COUNT; j++)
        {
            A[i][j] = M[j][i];
        }
    }

    for(int i = 0; i < AXIS_COUNT; i++) for(int j = 0; j < AXIS_COUNT; j++) M[i][j] = A[i][j];
}

void vecXmat(float* V, float M[AXIS_COUNT][AXIS_COUNT]) {
    float B[AXIS_COUNT];
    for(int i = 0; i < AXIS_COUNT; i++) B[i] = 0;
    for(int i = 0; i < AXIS_COUNT; i++)
        for(int j = 0; j < AXIS_COUNT; j++)
            B[i] += V[j] * M[i][j];

    for(int i = 0; i < AXIS_COUNT; i++) V[i] = B[i];

}

void vecPvec(float* A, float* B) {  
    for(int i = 0; i < AXIS_COUNT; i++) A[i] += B[i];
}

void vecXfloat(float* A, float kb) {
    for(int i = 0; i < AXIS_COUNT; i++) A[i] *= kb;
}