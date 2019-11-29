

#include "Ogre.h"
#include <iostream>
#include <vector> 

#include "raisim/RaiCameraMan.hpp"
#include "interfaceClasses.hpp"
#include "raisim/World.hpp"

namespace raisim
{

class SkeletonMesh
{
public:
    SkeletonMesh()
    {
        rotationOffset.setIdentity();
        scale = {1.0, 1.0, 1.0};
        offset = {.0, .0, .0};
        worldMode = false;
    }

    raisim::ArticulatedSystem *m_as;

    Ogre::SceneNode *graphics = nullptr;
    Vec<3> scale, offset;
    Mat<3,3> rotationOffset;

    unsigned long int group = 1ul << 4;
    std::string name;
    std::string meshName;

    // ogre skeleton props
    Ogre::SkeletonInstance *m_pSkeleton = nullptr;
    
    std::vector<Ogre::Bone *> m_Bones;
    std::vector<size_t> m_BodyIdxs;
    Ogre::Bone * m_rootBone = nullptr;
    size_t m_BodyCount;

    std::vector<ArticulatedSystem::JointRef> m_Joints;

    size_t root_idx = 0;
    bool worldMode = true;



    void setArticulateSystem(raisim::ArticulatedSystem *as, bool inWorld=false)
    {
        // the raisim character
        m_as = as;

        m_Bones.clear();
        m_BodyIdxs.clear();
        m_Joints.clear();
        m_BodyCount = 0;

        // search root bone, assume only one root for the skeleton
        for (auto rbone : m_pSkeleton->getRootBoneIterator())
        {
            m_rootBone = rbone;
            std::cout << "root bone: " << rbone->getName() << std::endl;
            m_rootBone->setManuallyControlled(true);
        }

        // loop character bodies
        // for (auto body_name : as->getBodyNames())
        for (auto frame_ : as->getFrames())
        {
            std::string body_name = frame_.name;

            // find the matched joint by name except root bone
            for (auto bone : m_pSkeleton->getBones())
            {

                if (m_rootBone == bone)
                    continue;

                std::string str_ = bone->getName();
                if (str_.substr(str_.find_last_of(':') + 1) == body_name)
                {
                    m_Joints.push_back(as->getJoint(body_name));
                    m_Bones.push_back(bone);

                    auto body_id = as->getBodyIdx(body_name);
                    m_BodyIdxs.push_back(body_id);

                    bone->setManuallyControlled(true);

                    // for world rotation mode
                    std::cout << body_name << " -> " << str_ << ", " << body_id << std::endl;

                    m_BodyCount++;
                    break;
                }
            }
        }

        setRotationMode(inWorld);
    }


    void setRotationMode(bool inWorld){
        worldMode = inWorld;
        
        for (auto bone: m_Bones){
            bone->setInheritOrientation(!inWorld);
        }
    }

    void syncStateToVisual()
    {

        Mat<3, 3> bodyRotation, rot;
        Vec<4> quat;
        Vec<3> pos, offsetInWorld;

        graphics->setScale(float(scale[0]), float(scale[1]), float(scale[2]));

        // copy root transform
        m_as->getBasePosition(pos);
        m_as->getBaseOrientation(bodyRotation);

        // matvecmul(bodyRotation, offset, offsetInWorld);
        offsetInWorld = offset;
        matmul(bodyRotation, rotationOffset, rot);


        graphics->setPosition(float(pos[0]+offsetInWorld[0]), 
                              float(pos[1]+offsetInWorld[1]), 
                              float(pos[2]+offsetInWorld[2]));
    
        rotMatToQuat(rot, quat);

        // graphics->setOrientation(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]));
        m_rootBone->setOrientation(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]));


        // copy joint transform
        raisim::VecDyn coord = m_as->getGeneralizedCoordinate(); 
        for (int i = 0; i < m_BodyCount; ++i)
        {

            if (worldMode){
                m_Joints[i].getOrientation(bodyRotation);
                rot = bodyRotation;
                rotMatToQuat(rot, quat);
            }
            else{
                for (auto k=0;k<4;k++){
                    quat[k] = coord[3+4*(m_BodyIdxs[i])+k];
                }
            }
            // m_as->getFrameOrientation(m_BodyIdxs[i], bodyRotation);
            
            m_Bones[i]->setOrientation(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]));
        }
    }

    void update()
    {
        syncStateToVisual();
    }
};

}