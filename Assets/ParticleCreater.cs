using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


public class object_data{
    // particle
    public Vector3 position_vector;
    public Vector3 velocity_vector;
    public Vector3 force_vector;

    public float particle_mass;

    public Vector3 particle_scale;
    public Quaternion rotation;

    public Matrix4x4 matrix
        {
            get
            {
                return Matrix4x4.TRS(position_vector, rotation, particle_scale);
            }
        }

    public object_data(Vector3 position_vector,Vector3 velocity_vector, Vector3  force_vector, float particle_mass, Vector3 particle_scale, Quaternion rotation){
        //Constructor

        //kinematics
        this.position_vector=position_vector;
        this.velocity_vector=velocity_vector;
        this.force_vector=force_vector;
        this.particle_mass=particle_mass;

        this.particle_scale=particle_scale;
        this.rotation=rotation;

    }//end of constructor

    //Setter and Getter methods

    // position vector setter getter method
    public Vector3 get_position_vector(){
        return this.position_vector;
    }

    public void set_position_vector(Vector3 new_position_vector){
        this.position_vector=new_position_vector;
    }

    // veclocity vector setter getter method

    public Vector3 get_velocity_vector(){
        return this.velocity_vector;
    }

    public void set_velocity_vector(Vector3 new_velocity_vector){
        this.velocity_vector=new_velocity_vector;
    }

    // force vector setter getter methods
    public Vector3 get_force_vector(){
        return this.force_vector;
    }

    public void set_force_vector(Vector3 new_force_vector){
        this.force_vector=new_force_vector;
    }

    // particle mass getter setter method
    public float get_particle_mass(){
        return this.particle_mass;
    }
    public void set_particle_mass(float new_particle_mass){
        this.particle_mass=new_particle_mass;
    }

    // particle scale getter setter method
    public Vector3 get_particle_scale(){
        return this.particle_scale;
    }
    public void set_particle_scale(Vector3 new_particle_scale){
        this.particle_scale=new_particle_scale;
    }

}



public class ParticleCreater : MonoBehaviour
{

    public Mesh particle_mesh;
    public Material particle_material;

    public int instances=600;
    public int batchLoop=100;
    public Vector3 max_position=new Vector3(100.0f,100.0f,100.0f);

    public Vector3 max_velocity=new Vector3(0.0f,0.0f,0.0f);
    public Vector3 max_force=new Vector3(0.0f,0.0f,0.0f);
    public Vector3 particle_scale=new Vector3(1.0f,1.0f,1.0f);
    public float particle_mass=1.0f;

    public float gravity_source_mass=1000.0f;
    public Vector3 gravity_source_scale=new Vector3(10.0f,10.0f,10.0f);
    public Vector3 gravity_source_position=new Vector3(0.0f,0.0f,0.0f);

    public float virtual_gravity_const=1.0f;

    public float time_step=0.5f;




    //instance list
    private List<List<object_data>> batches = new List<List<object_data>>();
    private List<object_data> source_bathc=new List<object_data>();


    // Start is called before the first frame update
    void Start()
    {
        // add gravtiation source object
        add_source_object(source_bathc,0);


        // add particles
        int batchIndexNum = 0;
        List<object_data> currBatch = new List<object_data>();


        for (int i = 0; i < instances; i++)
        {

            add_object(currBatch, i);
            batchIndexNum++;
            if (batchIndexNum >= batchLoop)
            {
                batches.Add(currBatch);
                currBatch = BuildNewBatch();
                batchIndexNum = 0;
            }
        }

    }

    // Update is called once per frame
    void Update()
    {
        RenderBatches();
        particle_versus_source(batches,source_bathc);
    }


    //Numerical part

    void particle_versus_source(List<List<object_data>> batches, List<object_data> source_bathc){
        object_data source_object=source_bathc[0];
        Vector3 source_object_positions=source_object.get_position_vector();
        float source_object_mass=source_object.get_particle_mass();

        for (int i=0 ;i<batches.Count;i++){
            for(int j=0;j<batches[0].Count;j++){
                //particle data
                object_data particle_object=batches[i][j];
                Vector3 particle_object_positions=particle_object.get_position_vector();
                float particle_object_mass=particle_object.get_particle_mass();
                Vector3 particle_object_velocity_vector=particle_object.get_velocity_vector();
                Vector3 particle_object_force_vector=particle_object.get_force_vector();


                verlet_integration(virtual_gravity_const,source_object_positions,particle_object_positions,source_object_mass,particle_object_mass,particle_object_velocity_vector,particle_object_force_vector,particle_object);
            }
        }
    }

    //Not exactly verlet integration
    public void verlet_integration(float virtual_gravity_const,Vector3 source_position, Vector3 particle_position,float source_mass,float particle_mass,Vector3 particle_object_velocity_vector,Vector3 particle_object_force_vector,object_data particle_object){
        float del_time_sq=time_step*time_step;

        //calculations

        particle_object_force_vector=force_direction(virtual_gravity_const,source_position,particle_position,source_mass,particle_mass);


        //position update
        particle_position[0]+=particle_object_velocity_vector[0]*time_step+(0.5f)*(particle_object_force_vector[0]/particle_mass)*del_time_sq;
        particle_position[1]+=particle_object_velocity_vector[1]*time_step+(0.5f)*(particle_object_force_vector[1]/particle_mass)*del_time_sq;
        particle_position[2]+=particle_object_velocity_vector[2]*time_step+(0.5f)*(particle_object_force_vector[2]/particle_mass)*del_time_sq;


        //velcity update new
        particle_object_velocity_vector[0]+=(particle_object_force_vector[0]/particle_mass)*time_step;
        particle_object_velocity_vector[1]+=(particle_object_force_vector[1]/particle_mass)*time_step;
        particle_object_velocity_vector[2]+=(particle_object_force_vector[2]/particle_mass)*time_step;

        if (check_collision(source_position,particle_position)==true){
            //Vector3 reduced_particle_object_velocity_vector=-0.2f*particle_object_velocity_vector;
            particle_object_velocity_vector=-(particle_object_velocity_vector-0.1f*particle_object_velocity_vector);
        }

        particle_object.set_force_vector(particle_object_force_vector);
        particle_object.set_position_vector(particle_position);
        particle_object.set_velocity_vector(particle_object_velocity_vector);
    }


    public bool check_collision(Vector3 source_position, Vector3 particle_position){
        Vector3 direction=source_position-particle_position;
        float distance=direction.magnitude;
        float min_coll_distance=0.5f*(particle_scale.x+gravity_source_scale.x);

        if(Mathf.Abs(min_coll_distance-distance)<=2.0f){
            return true;
        }
        return false;
    }

    public Vector3 force_direction(float virtual_gravity_const,Vector3 source_position, Vector3 particle_position,float source_mass,float particle_mass){
        Vector3 direction=source_position-particle_position;
        float distance=direction.magnitude;

        float force_mag=virtual_gravity_const*(source_mass*particle_mass)/(Mathf.Pow(distance,2));
        Vector3 force_vector=direction.normalized*force_mag;

        return force_vector;
    }





    // add and render objects
    private void add_object(List<object_data> currBatch, int i)

    {
        //Debug.Log("add_object function has started: Object added index: ");
        Vector3 position = new Vector3(Random.Range(-max_position.x, max_position.x), Random.Range(-max_position.y, max_position.y), Random.Range(-max_position.z, max_position.z));
        Vector3 velocity = new Vector3(Random.Range(-max_velocity.x, max_velocity.x), Random.Range(-max_velocity.y, max_velocity.y), Random.Range(-max_velocity.z, max_velocity.z));
        Vector3 force_vector=new Vector3(Random.Range(-max_force.x, max_force.x), Random.Range(-max_force.y, max_force.y), Random.Range(-max_force.z, max_force.z));

        currBatch.Add(new object_data(position, velocity, force_vector , particle_mass, particle_scale, Quaternion.identity));

    }


    private void add_source_object(List<object_data> soruce_batch, int i){
         soruce_batch.Add(new object_data(gravity_source_position, new Vector3(0,0,0), new Vector3(0,0,0) , gravity_source_mass, gravity_source_scale, Quaternion.identity));
    }
    private List<object_data> BuildNewBatch()
        {
            return new List<object_data>();
        }


        private void RenderBatches(){
            // render source object
        Graphics.DrawMeshInstanced(particle_mesh, 0, particle_material, source_bathc.Select((a) => a.matrix).ToList());

        // render particle objects
        foreach(var batch in batches){
            Graphics.DrawMeshInstanced(particle_mesh, 0, particle_material, batch.Select((a) => a.matrix).ToList());
        }



    }
}
