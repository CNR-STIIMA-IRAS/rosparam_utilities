#include <rosparam_utilities/rosparam_utilities.h>


namespace rosparam_utilities 
{

  
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, unsigned int& val )
  {
    XmlRpc::XmlRpcValue config ( node );
    
    switch ( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:
      {
        bool   _val = static_cast< bool   > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
      {
        int    _val = static_cast< int    > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeDouble:
      {
        double _val = static_cast< double > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeString:
      {
        unsigned int x;
        std::stringstream ss;
        ss << std::hex <<  static_cast< std::string > ( config );
        ss >> x;
        // output it as a signed type
        std::cout << static_cast<int> ( x ) << std::endl;
        val = x;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeBase64:
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error ( "Type inconsistency" );
        break;
    }
  }
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, bool& val )
  {
    XmlRpc::XmlRpcValue config ( node );
    
    switch ( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:
      {
        bool   _val = static_cast< bool   > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
      {
        int    _val = static_cast< int    > ( config );
        val = _val > 0;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeDouble:
      {
        double _val = static_cast< double > ( config );
        val = _val > 0;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeBase64:
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeString:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error ( "Type inconsistency" );
        break;
    }
  }
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, double& val )
  {
    XmlRpc::XmlRpcValue config ( node );
    
    switch ( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:
      {
        bool   _val = static_cast< bool   > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
      {
        int    _val = static_cast< int    > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeDouble:
      {
        double _val = static_cast< double > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeBase64:
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeString:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error ( "Type inconsistency" );
        break;
    }
  }
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, int& val )
  {
    XmlRpc::XmlRpcValue config ( node );
    
    switch ( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:
      {
        bool   _val = static_cast< bool   > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
      {
        int    _val = static_cast< int    > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeDouble:
      {
        double _val = static_cast< double > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeString:
      {
        std::string s = config;
        unsigned int x;
        std::stringstream ss;
        ss << std::hex <<  s;
        ss >> x;
        val = x;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeBase64:
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error ( "Type inconsistency" );
        break;
    }
  }
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, long unsigned int& val )
  {
    XmlRpc::XmlRpcValue config ( node );
    
    switch ( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:
      {
        bool   _val = static_cast< bool   > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
      {
        int    _val = static_cast< int    > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeDouble:
      {
        double _val = static_cast< double > ( config );
        val = _val;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeString:
      {
        unsigned int x;
        std::stringstream ss;
        ss << std::hex <<  static_cast< std::string > ( config );
        ss >> x;
        // output it as a signed type
        std::cout << static_cast<int> ( x ) << std::endl;
        val = x;
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeBase64:
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error ( "Type inconsistency" );
        break;
    }
  }
  void fromXmlRpcValue ( const XmlRpc::XmlRpcValue& node, std::string& val )
  {
    XmlRpc::XmlRpcValue config ( node );
    
    switch ( config.getType() )
    {
      case XmlRpc::XmlRpcValue::TypeBoolean:
      {
        bool   _val = static_cast< bool   > ( config );
        val = std::to_string ( _val );
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
      {
        int    _val = static_cast< int    > ( config );
        val = std::to_string ( _val );
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeDouble:
      {
        double _val = static_cast< double > ( config );
        val = std::to_string ( _val );
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeString:
      {
        std::string _val = static_cast< std::string > ( config );
        val = _val ;
        val.erase ( std::remove_if ( val.begin(), val.end(), isspace ), val.end() );
      }
      break;
      
      case XmlRpc::XmlRpcValue::TypeBase64:
      case XmlRpc::XmlRpcValue::TypeInvalid:
      case XmlRpc::XmlRpcValue::TypeDateTime:
      case XmlRpc::XmlRpcValue::TypeArray:
      case XmlRpc::XmlRpcValue::TypeStruct:
        throw std::runtime_error ( "Type inconsistency" );
        break;
    }
  }
  
  void toXmlRpcValue ( const int& t,  XmlRpc::XmlRpcValue& xml_value, std::string format )
  {
    if ( format == "dec" )
    {
      xml_value = ( int ) t;
    }
    else if ( format == "hex" )
    {
      std::stringstream s;
      s << "0x" << std::hex << t ;
      xml_value = s.str();
    }
    else if ( format == "bin" )
    {
      std::stringstream s;
      s << "0b" << std::bitset<32> ( t );
      xml_value = s.str();
    }
  }
  void toXmlRpcValue ( const std::string& t,  XmlRpc::XmlRpcValue& xml_value )
  {
    xml_value = ( std::string ) t;
  }
  void toXmlRpcValue ( const bool& t,  XmlRpc::XmlRpcValue& xml_value )
  {
    xml_value = ( bool ) t;
  }
  void toXmlRpcValue ( const double& t,  XmlRpc::XmlRpcValue& xml_value )
  {
    xml_value = ( double ) t;
  }
  
  
  
  bool check ( const XmlRpc::XmlRpcValue& config, const std::vector< std::string >& required_fields )
  {
    for ( std::vector< std::string >::const_iterator it = required_fields.begin(); it != required_fields.end(); it++ )
    {
      if ( !config.hasMember ( *it ) )
      {
        ROS_ERROR ( " CHECK XmlRpcValue -The field '%s' does not exist. ", it->c_str() );
        return false;
      }
    }
    
    return true;
  }
  

}
