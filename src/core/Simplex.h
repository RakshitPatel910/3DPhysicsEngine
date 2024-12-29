#pragma once

#include<vector>

#include "Vector3.h"

class Simplex
{
private:
    std::vector<Vector3> m_simplex;

public:
    Vector3& operator[](size_t i) const {
        return m_simplex[i];
    }

    size_t size() const {
        return m_simplex.size();
    }

    void addPoint(const Vector3& point){
        m_simplex.push_back(point);
    }

    void removePoint(const size_t i){
        m_simplex.erase(m_simplex.begin() + i);
    }

    void clear() {
        m_simplex.clear();
    }

    bool containsOrigin(Vector3& dir){
        if(m_simplex.size() == 2){ // line segment
            Vector3 A = m_simplex[1];
            Vector3 B = m_simplex[0];

            Vector3 AB = B - A;
            Vector3 AO = -A;

            // if( AO.angleBetween(AB) == 0.0f && AO.length() <= AB.length() ){
            //     return true;
            // }

            dir = AB.cross(AO).cross(AB);
            if(dir.length() < 1e-6f){
                dir = dir.normalized();
            }

            return false;
        }
        else if(m_simplex.size() == 3){ // triangle
            Vector3 A = m_simplex[2];
            Vector3 B = m_simplex[1];
            Vector3 C = m_simplex[0];

            Vector3 AB = B - A;
            Vector3 AC = C - A;
            Vector3 AO = -A;

            Vector3 normalABC = AB.cross(AC); // normal to plane formed by AB and AC

            // for understanding this easily use right hand thumb rule
            if(normalABC.cross(AC).dot(AO) > 0){ // origin is on the side of normal to plane made by normalABC and AC (not AC and normalABC)
                                                 // i.e on the side of AC, but outside triangle
                m_simplex.erase(m_simplex.begin() + 1); // removing B  {C, B, A} ==> {C, A}

                dir = AC.cross(AO).cross(AC).normalized();
            }
            else if(AB.cross(normalABC).dot(AO) > 0){ // i.e on the side of AB, but outside triangle
                m_simplex.erase(m_simplex.begin()); // removing C  {C, B, A} ==> {B, A}

                dir = AB.cross(AO).cross(AB).normalized();
            }
            else{ // origin is inside triangel, no necessarily on the same plane
                dir = normalABC;

                return true;
            }

            return false;
        }
        else f(m_simplex.size() == 4){ // tetrahedron
            Vector3 A = m_simplex[3];
            Vector3 B = m_simplex[2];
            Vector3 C = m_simplex[1];
            Vector3 D = m_simplex[0];
            
            Vector3 AB = B - A;
            Vector3 AC = C - A;
            Vector3 AD = D - A;
            Vector3 AO = -A;

            Vector3 normalABC = AB.cross(AC); // normal to plane formed by AB and AC, normal points outside of tetrahedron
            Vector3 normalADB = AD.cross(AB); // normal to plane formed by AD and AB, normal points outside of tetrahedron
            Vector3 normalACD = AC.cross(AD); // normal to plane formed by AC and AD, normal points outside of tetrahedron
            Vector3 normalBCD = (C - B).cross(D - B); // normal to plane formed by BC and BD, normal points outside of tetrahedron

            if(normalABC.dot(AO) > 0){
                m_simplex.erase(m_simplex.begin()); // removing D  {D, C, B, A} ==> {C, B, A}

                dir = normalABC;
            }
            else if(normalADB.dot(AO) > 0){
                m_simplex.erase(m_simplex.begin() + 1); // removing C  {D, C, B, A} ==> {D, B, A}

                dir = normalADB;
            }
            else if(normalACD.dot(AO) > 0){
                m_simplex.erase(m_simplex.begin() + 2); // removing B  {D, C, B, A} ==> {D, C, A}

                dir = normalACD;
            }
            else if(normalBCD.dot(AO) > 0){
                m_simplex.erase(m_simplex.begin() + 3); // removing A  {D, C, B, A} ==> {D, C, B}

                dir = normalBCD;
            }
            else{ // origin inside tetrahedron
                return true;
            }

            return false;
        }

        return false;
    }
};

