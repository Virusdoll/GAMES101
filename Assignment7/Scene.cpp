//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // check depth
    // if (depth <= 0)
    // {
    //     return Vector3f(0.0f);
    // }

    Intersection hit = intersect(ray);

    // no intersection
    if (!hit.happened)
    {
        return Vector3f(0.0f);
    }

    // indirectly hit a emitting object
    if (hit.m->hasEmission() && depth != 0)
    {
        return Vector3f(0.0f);
    }

    // directly hit a light source
    if (hit.m->hasEmission() && depth == 0)
    {
        return hit.m->getEmission();
    }
    
    Vector3f wi = ray.direction;
    Vector3f N = hit.normal;
    Vector3f p = hit.coords;
    Material m = *hit.m;
    Vector3f wo = m.sample(wi, N).normalized();
    
    // direct light
    Vector3f L_dir(0.0f);

    float pdf_light;
    Intersection inter;
    sampleLight(inter, pdf_light);

    Vector3f xx = inter.coords;
    Vector3f ws = (xx - p).normalized();
    Vector3f NN = inter.normal;
    Vector3f emit = inter.emit;

    if ((intersect(Ray(p, ws)).coords - xx).norm() < 0.001f)
    {
        L_dir = emit * m.eval(wi, ws, N) * dotProduct(ws, N) * dotProduct((p - xx).normalized(), NN) / std::pow((xx - p).norm(), 2) / pdf_light;
    }
    
    // indirect light
    Vector3f L_indir(0.0f);

    // Russian Roulette
    if (((float) rand()) / RAND_MAX < RussianRoulette)
    {   
        L_indir = castRay(Ray(p, wo), depth + 1) * m.eval(wi, wo, N) * dotProduct(wo, N) / m.pdf(wi, wo, N) / RussianRoulette;
    }
    return L_dir + L_indir;
}