#ifndef rosparam_utilities_201806130948
#define rosparam_utilities_201806130948

#include <ros/ros.h>
#include <XmlRpc.h>
#include <boost/array.hpp>
#include <bitset>

namespace rosparam_utilities
{
  
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, unsigned int&      val );
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, double&            val );
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, int&               val );
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, long unsigned int& val );
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, bool&              val );
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, std::string&       val );
  
  void toXmlRpcValue   ( const double&      t,  XmlRpc::XmlRpcValue& xml_value );
  void toXmlRpcValue   ( const int&         t,  XmlRpc::XmlRpcValue& xml_value, std::string format ="dec"  );
  void toXmlRpcValue   ( const std::string& t,  XmlRpc::XmlRpcValue& xml_value );
  void toXmlRpcValue   ( const bool&        t,  XmlRpc::XmlRpcValue& xml_value );
  
  
  bool check( const XmlRpc::XmlRpcValue& node, const std::vector< std::string >& required_fields );
  
  template <typename T> T fromXmlRpcValue(  const XmlRpc::XmlRpcValue& node, const std::string& field="", const std::string& log_key ="" )
  {
    T ret;
    bool ok = true;
    std::string error   = "Error. " 
    + std::string( field    != "" ? "Field: '"    + field     + "'." : "" )
    + std::string( log_key  != "" ? "Log Key: '"  + log_key   + "'." : "" );
    
    try 
    {
      XmlRpc::XmlRpcValue config( node );
      if( field != "" )
      {
        if( !config.hasMember( field.c_str() ) )
        {
          throw std::runtime_error( ("The param '" + field + "' is not in the rosparam server. ").c_str()  );
        }
        fromXmlRpcValue( config[field.c_str()], ret );
      }
      else
      {
        fromXmlRpcValue( config, ret );
      }
    }
    catch( XmlRpc::XmlRpcException& e )
    {
      error += e.getMessage();
      ok = false;
    }  
    catch( std::exception& e )
    {
      error += e.what();
      ok = false;
    }
    
    if(!ok)
      throw std::runtime_error( error.c_str() );
    
    return ret;
  }
  
  inline double               toDouble    ( const XmlRpc::XmlRpcValue& node, const std::string& field ="", const std::string& log ="" ) { return fromXmlRpcValue<double>      ( node, field, log ); }
  inline int                  toInt       ( const XmlRpc::XmlRpcValue& node, const std::string& field ="", const std::string& log ="" ) { return fromXmlRpcValue<int>         ( node, field, log ); }
  inline bool                 toBool      ( const XmlRpc::XmlRpcValue& node, const std::string& field ="", const std::string& log ="" ) { return fromXmlRpcValue<bool>        ( node, field, log ); }
  inline std::string          toString    ( const XmlRpc::XmlRpcValue& node, const std::string& field ="", const std::string& log ="" ) { return fromXmlRpcValue<std::string> ( node, field, log ); }
  
  
  template< class T> 
  inline bool getParam( const XmlRpc::XmlRpcValue& node,  T& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config( node );
    try 
    {
      ret = fromXmlRpcValue< T >( config );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted", log_key.c_str() );
      return false;
    }
    return true;
  }
  template< class T> 
  inline void extractParam( const XmlRpc::XmlRpcValue& node, const std::string& key,  T& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config( node );
    try 
    {
      ret = fromXmlRpcValue< T >( config, key );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted. %s", log_key.c_str(), e.what() );
      throw std::runtime_error("The node '" + log_key + "' is corrupted. "+ std::string( e.what() ) );
    }
  }
  
  template< class T> 
  inline bool getParam( const XmlRpc::XmlRpcValue& node,  std::string key,  T& ret )
  {
    XmlRpc::XmlRpcValue config( node );
    try 
    {
      ret = fromXmlRpcValue< T >( config, key );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "Error %s", e.what() );
      return false;
    }
    return true;
  }
  

  
  
  
  template< class T> 
  inline bool getParamVector ( const XmlRpc::XmlRpcValue& node,  std::vector< T >& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config( node );
    if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The node '%s' is not of type array. %d/%d", log_key.c_str(), int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
      return false;
    }
    
    ret.clear();
    for ( int j=0; j<config.size(); ++j )
    {
      try
      {
        T val;
        if(typeid(T) == typeid(double))
        {
          val = toDouble( config[j] );
        }
        else if(typeid(T) == typeid(std::string))
        {
          if( config[j].getType() == XmlRpc::XmlRpcValue::TypeString )
            fromXmlRpcValue( config[j], val );
          else
            throw std::runtime_error( ("Type Error: the node '" + log_key + "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" + std::to_string(  int( config[j].getType() ) ) ) .c_str() );
          
        }
        else
          fromXmlRpcValue( config[j], val );
        
        ret.push_back( val  );   
      }
      catch (std::exception& e )
      {
        ROS_ERROR_STREAM("Error: " << e.what() );
        return false;
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Wrong Format." );
        return false;
      }
    }
    return true;
  }
  
  template< class T, size_t n > 
  inline bool getParamArray ( const XmlRpc::XmlRpcValue& node,  boost::array< T, n >& ret, const std::string& log_key = "" )
  {
    std::vector< T > v;
    if(!getParamVector ( node, v,  log_key ) )
      return false;
    if( v.size() != n )
      return false;
    
    for( size_t i=0; i<n; i++ )
      ret[i] = v[i];
    return true;
  }
  
  template< class T> 
  inline bool getParamVector ( const XmlRpc::XmlRpcValue& node,  const std::string& key, std::vector< T >& ret  )
  {
    try 
    {
      if( !node.hasMember( key  ) )
      {
        ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
        return false;
      }
      
      XmlRpc::XmlRpcValue config( node );
      return getParamVector ( config[key],  ret, key  );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
      return false;
    }
    return true;
  }
  
  template< class T, size_t n > 
  inline bool getParamArray ( const XmlRpc::XmlRpcValue& node,  const std::string& key, boost::array< T, n >& ret  )
  {
    std::vector< T > v;
    if(!getParamVector ( node, key, v ) )
      return false;
    if( v.size() != n )
      return false;
    
    for( size_t i=0; i<n; i++ )
      ret[i] = v[i];
    return true;
  }
  
  template< class T >
  inline bool getParamMatrix ( const XmlRpc::XmlRpcValue& node, std::vector< std::vector< T > >& ret, const std::string& log_key = "" )
  {
    XmlRpc::XmlRpcValue config(node);
    if ( config.getType() != XmlRpc::XmlRpcValue::TypeArray )
    {
      ROS_ERROR ( "The node '%s' is not of type array. %d/%d", log_key.c_str(), int( config.getType()), int( XmlRpc::XmlRpcValue::TypeArray ) );
      return false;
    }
    
    ret.clear();
    for ( auto i=0; i<config.size(); ++i )
    {
      
      XmlRpc::XmlRpcValue row = config[i];
      if ( row.getType() != XmlRpc::XmlRpcValue::TypeArray )
      {
        ROS_ERROR ( "The row[%d] is not an array.", i );
        return false;
      }
      std::vector< T > vct;
      for ( int j=0; j<row.size(); ++j )
      {
        T value;
        try
        {
          XmlRpc::XmlRpcValue rpcval = row[j];
          if(typeid(T) == typeid(double))
          {
            value = toDouble( row[j] );
          }
          else if(typeid(T) == typeid(std::string))
          {
            if( row[j].getType() == XmlRpc::XmlRpcValue::TypeString )
              fromXmlRpcValue( row[j], value );
            else
              throw std::runtime_error( ("Type Error: the node '" + log_key + "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" + std::to_string(  int( row[j].getType() ) ) ).c_str() );
            
          }
          else
          {
            fromXmlRpcValue( row[j], value );
          }   
        }
        catch (std::exception& e )
        {
          ROS_ERROR_STREAM("Error: " << e.what() );
          if (typeid(T) == typeid(double))
            ROS_ERROR_STREAM("you have to specify the fractional part even if it is zero");
          return false;
        }
        catch (...)
        {
          if (typeid(T) == typeid(double))
            ROS_ERROR_STREAM("you have to specify the fractional part even if it is zero");
          return false;
        }
        vct.push_back ( value );
      }
      ret.push_back ( vct );
    }
    return true;
  }
  
  
  template< class T> 
  inline bool getParamMatrix ( const XmlRpc::XmlRpcValue& node,  const std::string& key, std::vector< T >& ret  )
  {
    try 
    {
      if( !node.hasMember( key  ) )
      {
        ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
        return false;
      }
      
      XmlRpc::XmlRpcValue config( node );
      return getParamMatrix ( config[key],  ret, key  );
    }
    catch( std::exception& e )
    {
      ROS_ERROR ( "The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid( T ).name() );
      return false;
    }
    return true;
  }
  
  
  
  template<typename T>
  bool getParam(const ros::NodeHandle& nh, const std::string& key, T& ret)
  {
    bool res=nh.getParam(key,ret);
    if (!res)
      ROS_WARN_STREAM("Parameter '" << key << "' not found!");
    
    ROS_DEBUG_STREAM("Parameter '" << key << "' loaded!");
    return res;
  }
  
  template< class T >
  inline bool getParamMatrix ( const ros::NodeHandle& nh,  const std::string& key, std::vector< std::vector< T > >& ret )
  {
    XmlRpc::XmlRpcValue config;
    nh.getParam(key,config);
    return getParamMatrix(config,ret, key);
  }
  
  
  template< class T >
  inline bool setParam ( ros::NodeHandle& nh, const std::string& key, const std::vector< std::vector< T > >& mtx )
  {
    XmlRpc::XmlRpcValue data_;
    int iRow = 0;
    for ( auto itRow=mtx.begin(); itRow!=mtx.end(); itRow++ )
    {
      int iEl = 0;
      XmlRpc::XmlRpcValue row_; 
      for ( auto itCol=(*itRow).begin(); itCol!=(*itRow).end(); itCol++ )
        row_[iEl++] = *itCol; 
      
      data_[iRow++] = (XmlRpc::XmlRpcValue)row_;
    }
    
    nh.setParam( key, data_ );
    return true;
  }
  
};

#endif
