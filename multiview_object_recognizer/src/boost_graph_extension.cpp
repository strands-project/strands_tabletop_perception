#include "boost_graph_extension.h"

struct my_graph_writer
{
	void
	operator() ( std::ostream& out ) const
	{
		out << "node [shape=circle color=blue]" << std::endl;
		// just an example, showing that local options override global
		out << "edge [color=red]" << std::endl;
	}
} myGraphWrite;

struct my_edge_writer
{
	my_edge_writer ( Graph& g_ ) :
		g ( g_ )
	{
	}
	;
	template<class Edge>
	void
	operator() ( std::ostream& out, Edge e )
	{
		// just an example, showing that local options override global
		out << " [color=purple]" << std::endl;
		out << " [label=\"" << g[e].edge_weight << boost::filesystem::path ( g[e].model_name ).stem ().string () << "\"]" << std::endl;
	}
	;
	Graph g;
};

struct my_node_writer
{
	my_node_writer ( Graph& g_ ) :
		g ( g_ )
	{
	}
	;
	template<class Vertex>
	void
	operator() ( std::ostream& out, Vertex v )
	{
		out << " [label=\"" << g[v].view_id_ << "(" << boost::filesystem::path ( g[v].view_id_ ).stem ().string () << ")\"]"
		    << std::endl;
		out << " [file=\"" << g[v].view_id_ << "\"]" << std::endl;
		out << " [index=\"" << g[v].view_id_ << "\"]" << std::endl;

		for ( std::vector<Hypothesis>::iterator it_hyp = g[v].hypothesis.begin (); it_hyp != g[v].hypothesis.end (); ++it_hyp )
		{
		    out << " [hypothesis_model_id=\"" << it_hyp->model_id_ << "\"]" << std::endl;
		    out << " [hypothesis_transform=\"" << it_hyp->transform_ ( 0, 0 ) << " " << it_hyp->transform_ ( 0, 1 ) << " " << it_hyp->transform_ ( 0, 2 )
		        << " " << it_hyp->transform_ ( 0, 3 ) << " " << it_hyp->transform_ ( 1, 0 ) << " " << it_hyp->transform_ ( 1, 1 ) << " "
		        << it_hyp->transform_ ( 1, 2 ) << " " << it_hyp->transform_ ( 1, 3 ) << " " << it_hyp->transform_ ( 2, 0 ) << " " << it_hyp->transform_ ( 2, 1 )
		        << " " << it_hyp->transform_ ( 2, 2 ) << " " << it_hyp->transform_ ( 2, 3 ) << " " << it_hyp->transform_ ( 3, 0 ) << " "
		        << it_hyp->transform_ ( 3, 1 ) << " " << it_hyp->transform_ ( 3, 2 ) << " " << it_hyp->transform_ ( 3, 3 ) << " " << "\"]" << std::endl;
		    out << " [hypothesis_origin=\"" << it_hyp->origin_ << "\"]" << std::endl;
		    out << " [hypothesis_verified=\"" << it_hyp->verified_ << "\"]" << std::endl;
		}
	}
	;
	Graph g;
};

void outputgraph ( Graph& map, const char* filename )
{
	std::ofstream gout;
	gout.open ( filename );
	write_graphviz ( gout, map, my_node_writer ( map ), my_edge_writer ( map ), myGraphWrite );
}

View::View ()
{
    pScenePCl.reset ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pScenePCl_f.reset ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pSceneNormals.reset ( new pcl::PointCloud<pcl::Normal> );
    //    pSceneXYZRGBNormal.reset ( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    //pScenePCl_f_ds.reset ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pIndices_above_plane.reset ( new pcl::PointIndices );
    pSignatures.reset ( new pcl::PointCloud<FeatureT> );
    has_been_hopped_ = false;
    cumulative_weight_to_new_vrtx_ = 0;
}

Hypothesis::Hypothesis ( const std::string model_id, const Eigen::Matrix4f transform, const std::string origin, const bool extended, const bool verified )
{
    model_id_ = model_id;
    transform_ = transform;
    origin_ = origin;
    extended_ = extended;
    verified_ = verified;
}

myEdge::myEdge()
{
    edge_weight = std::numeric_limits<float>::max ();
    model_name = "";
    source_id = "";
    target_id = "";
    edge_weight_has_been_calculated_ = false;
    std::vector <cv::DMatch> matches;
}

//std::vector<Vertex> my_node_reader ( std::string filename, Graph &g )
//{
//    std::string fn, model_id, line, tf_str, origin, verified;
//    Eigen::Matrix4f tf;
//    std::ifstream myfile;
//    std::vector<Vertex> vertices_temp_v;

//    myfile.open ( filename.c_str () );

//    if ( myfile.is_open () )
//    {
//        while ( myfile.good () )
//        {
//            std::getline ( myfile, line );

//            int found = -1;
//            std::string searchstring ( "[file=\"" );
//            found = line.find ( searchstring );

//            if ( found > -1 )
//            {
//                Vertex v = boost::add_vertex ( g );
//                vertices_temp_v.push_back ( v );

//                fn = line.substr ( found + searchstring.length () );
//                fn.erase ( fn.end () - 2, fn.end () );

//                int read_state = 0;
//                while ( myfile.good () && read_state > -1 )
//                {
//                    std::getline ( myfile, line );

//                    searchstring = ";";
//                    found = line.find ( searchstring );
//                    if ( found > -1 )
//                    {
//                        read_state = -1;
//                        break;
//                    }
//                    else
//                    {
//                        searchstring = "[hypothesis_model_id=\"";
//                        found = line.find ( searchstring );
//                        if ( found > -1 )
//                        {
//                            model_id = line.substr ( found + searchstring.length () );
//                            model_id.erase ( model_id.end () - 2, model_id.end () );
//                            read_state++;
//                        }

//                        searchstring = "[hypothesis_transform=\"";
//                        found = line.find ( searchstring );
//                        if ( found > -1 )
//                        {
//                            tf_str = line.substr ( found + searchstring.length () );
//                            tf_str.erase ( tf_str.end () - 2, tf_str.end () );

//                            std::stringstream ( tf_str ) >> tf ( 0, 0 ) >> tf ( 0, 1 ) >> tf ( 0, 2 ) >> tf ( 0, 3 ) >> tf ( 1, 0 ) >> tf ( 1, 1 ) >> tf ( 1, 2 ) >> tf ( 1, 3 )
//                                                         >> tf ( 2, 0 ) >> tf ( 2, 1 ) >> tf ( 2, 2 ) >> tf ( 2, 3 ) >> tf ( 3, 0 ) >> tf ( 3, 1 ) >> tf ( 3, 2 ) >> tf ( 3, 3 );
//                            read_state++;

//                        }
//                        searchstring = "[hypothesis_origin=\"";
//                        found = line.find ( searchstring );
//                        if ( found > -1 )
//                        {
//                            origin = line.substr ( found + searchstring.length () );
//                            origin.erase ( origin.end () - 2, origin.end () );
//                            read_state++;

//                            searchstring = "[hypothesis_verified=\"";
//                            found = line.find ( searchstring );
//                            if ( found > -1 )
//                            {
//                                verified = line.substr ( found + searchstring.length () );
//                                verified.erase ( verified.end () - 2, verified.end () );
//                                read_state++;
//                            }
//                        }
//                    }
//                    if ( read_state >= 4 )
//                    {
//                        read_state = 0;
//                        Hypothesis hypothesis ( model_id, tf, origin, false, atoi ( verified.c_str() ) );
//                        g[v].hypothesis.push_back ( hypothesis );

//                        g[v].scene_filename = fn;
//                        g[v].pScenePCl.reset ( new pcl::PointCloud<pcl::PointXYZRGB> );
//                        pcl::io::loadPCDFile ( g[v].scene_filename, * ( g[v].pScenePCl ) );

//                    }
//                }
//            }
//        }
//        myfile.close ();
//    }
//    return vertices_temp_v;
//}
