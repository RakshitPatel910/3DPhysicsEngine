#pragma once

#include<cstring>
#include<Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Vector3.h"

class Matrix4
{
// private:
public:
    float m[16];
    Matrix4(){
        std::memset(m, 0, sizeof(m));
        for( int i = 0; i < 4; i++ ){
            m[i*4 + i] = 1.0f;
        }
    }

    Matrix4 operator*(const Matrix4& B) const {
        Matrix4 mat;

        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                mat.m[i*4 + j] = 0;

                for(int k = 0; k < 4; k++){
                    mat.m[i*4 + j] += m[i * 4 + k] * B.m[k * 4 + j];
                }
            }
        }

        mat.m[12] = m[12] + B.m[12];
        mat.m[13] = m[13] + B.m[13];
        mat.m[14] = m[14] + B.m[14];

        return mat;
    }

    // Matrix4 getM(){ return m; }

    Matrix4 transpose() const {
        Matrix4 mat;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                mat.m[i * 4 + j] = m[j * 4 + i];
            }
        }

        return mat;
    }

     Matrix4 inverse() const {
        Matrix4 L, U;
        Matrix4 inverseMat;

        // Step 1: Perform LU Decomposition
        if (!LUDecompose(L, U)) {
            // Matrix is singular and cannot be inverted
            // std::cerr << "Matrix is singular and cannot be inverted!" << std::endl;
            return inverseMat;  // Return a zero matrix or handle as needed
        }

        // Step 2: Solve L * Y = B (where B is the identity matrix)
        // Step 3: Solve U * X = Y (where X is the inverse of the matrix)
        inverseMat = solveLU(L, U);

        return inverseMat;
    }

    glm::mat4 toGlm() const {
        return glm::mat4(
            m[0], m[1], m[2], m[3],
            m[4], m[5], m[6], m[7],
            m[8], m[9], m[10], m[11],
            m[12], m[13], m[14], m[15]
        );
    }

    Eigen::Matrix3f toEigenMatrix3f() const {
        Eigen::Matrix3f result;

        // Extract the top-left 3x3 block and assign to Eigen::Matrix3f
        result(0, 0) = m[0];
        result(0, 1) = m[1];
        result(0, 2) = m[2];

        result(1, 0) = m[4];
        result(1, 1) = m[5];
        result(1, 2) = m[6];

        result(2, 0) = m[8];
        result(2, 1) = m[9];
        result(2, 2) = m[10];

        return result;
    }

    static Matrix4 getTranslationMatrix(const Vector3& tVec){
        Matrix4 mat;

        mat.m[12] = tVec.getX();
        mat.m[13] = tVec.getY();
        mat.m[14] = tVec.getZ();


        return mat;
    }

    static Matrix4 getScalingMAtrix(const Vector3& sVec){
        Matrix4 mat;

        mat.m[0] = sVec.getX();
        mat.m[5] = sVec.getY();
        mat.m[10] = sVec.getZ();
    
        return mat;
    }

    static Matrix4 getRotationXMatrix(float angleX){
        Matrix4 mat;

        float cosX = std::cos(angleX);
        float sinX = std::sin(angleX);

        mat.m[5] = cosX;
        mat.m[6] = -sinX;
        mat.m[9] = sinX;
        mat.m[10] = cosX;

        return mat;
    }

    static Matrix4 getRotationYMatrix(float angleY){
        Matrix4 mat;

        float cosY = std::cos(angleY);
        float sinY = std::sin(angleY);

        mat.m[0] = cosY;
        mat.m[2] = -sinY;
        mat.m[8] = sinY;
        mat.m[10] = cosY;

        return mat;
    }

    static Matrix4 getRotationZMatrix(float angleZ){
        Matrix4 mat;

        float cosZ = std::cos(angleZ);
        float sinZ = std::sin(angleZ);

        mat.m[0] = cosZ;
        mat.m[2] = -sinZ;
        mat.m[4] = sinZ;
        mat.m[5] = cosZ;

        return mat;
    }
    
    static Matrix4 getRotationMatrixByAngleOnAxis(float angle, const Vector3& axis){
        Matrix4 mat;

        Vector3 nAxis = axis.normalized(); // Normalize the axis
        float x = nAxis.getX(), y = nAxis.getY(), z = nAxis.getZ();

        float c = cos(angle);           // Cosine of angle
        float s = sin(angle);           // Sine of angle
        float oneMinusC = 1.0f - c;     // 1 - cos(angle)

        // Populate rotation matrix using the axis-angle formula
        mat.m[0] = c + x * x * oneMinusC;
        mat.m[1] = x * y * oneMinusC - z * s;
        mat.m[2] = x * z * oneMinusC + y * s;

        mat.m[4] = y * x * oneMinusC + z * s;
        mat.m[5] = c + y * y * oneMinusC;
        mat.m[6] = y * z * oneMinusC - x * s;

        mat.m[8] = z * x * oneMinusC - y * s;
        mat.m[9] = z * y * oneMinusC + x * s;
        mat.m[10] = c + z * z * oneMinusC;

        return mat;
    }

    // Do i need to add translation part for transforming Vector3 ????
    Vector3 transformVec(const Vector3& vec) const {
        return Vector3(
            m[0] * vec.getX() + m[4] * vec.getY() + m[8]  * vec.getZ(),
            m[1] * vec.getX() + m[5] * vec.getY() + m[9]  * vec.getZ(),
            m[2] * vec.getX() + m[6] * vec.getY() + m[10] * vec.getZ()
        );
    }




    bool LUDecompose(Matrix4& L, Matrix4& U) const {
        // Initialize L and U matrices
        L = Matrix4();
        U = *this;

        for (int i = 0; i < 4; i++) {
            // Calculate U
            for (int j = i + 1; j < 4; j++) {
                if (U.m[i * 4 + i] == 0) return false; // Matrix is singular

                float factor = U.m[j * 4 + i] / U.m[i * 4 + i];
                L.m[j * 4 + i] = factor;  // Store multipliers in L

                for (int k = i; k < 4; k++) {
                    U.m[j * 4 + k] -= factor * U.m[i * 4 + k]; // Update U
                }
            }
        }

        // Set diagonal of L to 1
        for (int i = 0; i < 4; i++) {
            L.m[i * 4 + i] = 1.0f;
        }

        return true;
    }

    // Solve for X in the equation L * X = B and U * X = Y
    Matrix4 solveLU(const Matrix4& L, const Matrix4& U) const {
        Matrix4 result;
        Matrix4 identity;

        // Step 1: Solve L * Y = B (where B is the identity matrix)
        for (int i = 0; i < 4; i++) {
            identity.m[i * 4 + i] = 1.0f;
        }

        // Forward substitution to solve L * Y = I (where I is the identity matrix)
        Matrix4 Y = forwardSubstitution(L, identity);

        // Step 2: Solve U * X = Y (using backward substitution)
        result = backwardSubstitution(U, Y);

        return result;
    }

    // Forward substitution: Solve L * X = B
    Matrix4 forwardSubstitution(const Matrix4& L, const Matrix4& B) const {
        Matrix4 X;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                X.m[i * 4 + j] = B.m[i * 4 + j];
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (i > j) {
                    for (int k = 0; k < 4; k++) {
                        X.m[i * 4 + j] -= L.m[i * 4 + k] * X.m[k * 4 + j];
                    }
                }
            }
        }

        return X;
    }

    // Backward substitution: Solve U * X = Y
    Matrix4 backwardSubstitution(const Matrix4& U, const Matrix4& Y) const {
        Matrix4 X;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                X.m[i * 4 + j] = Y.m[i * 4 + j];
            }
        }

        for (int i = 3; i >= 0; i--) {
            for (int j = 0; j < 4; j++) {
                if (i < j) {
                    for (int k = 0; k < 4; k++) {
                        X.m[i * 4 + j] -= U.m[i * 4 + k] * X.m[k * 4 + j];
                    }
                    X.m[i * 4 + j] /= U.m[i * 4 + i];
                }
            }
        }

        return X;
    }

    void printMat() const {
        std::cout << "Transformation Matrix:" << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << "[" << m[i * 4 + 0] << ", " << m[i * 4 + 1] << ", "
                      << m[i * 4 + 2] << ", " << m[i * 4 + 3] << "]" << std::endl;
        }
    }

};

