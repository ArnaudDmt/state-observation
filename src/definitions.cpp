#include <fstream>
#include <sstream>
#include <stdexcept>
#include <state-observation/tools/definitions.hpp>


namespace stateObservation
{

  namespace detail
  {
    const bool defaultTrue = true;
    const char* defaultErrorMSG = "The Object is not initialized. \
         If this happened during initialization then run command chckitm_set() \
         to switch it to set. And if the initialization is incomplete, run \
         chckitm_reset() afterwards.";
    const std::runtime_error defaultException(defaultErrorMSG);
    const std::exception* defaultExcepionAddr=&defaultException;


    void defaultSum(const  Vector& stateVector, const Vector& tangentVector, Vector& result)
    {
      result.noalias() = stateVector + tangentVector;
    }

    void defaultDifference(const  Vector& stateVector1, const Vector& stateVector2, Vector& difference)
    {
      difference.noalias() = stateVector1 - stateVector2;
    }

  }

  namespace tools
  {
    std::string matrixToString(const Matrix& mat)
    {
      std::stringstream ss;
      ss << mat;
      return ss.str();
    }

    std::string vectorToString(const Vector& v)
    {
      return matrixToString(v.transpose());
    }

    Matrix stringToMatrix(const std::string& str, unsigned rows, unsigned cols)
    {
      Matrix m(Matrix::Zero(rows,cols));
      std::stringstream ss;
      ss << str;

      for (size_t i = 0 ; i<rows; ++i)
      {
        for (size_t j = 0 ; j<cols; ++j)
        {
          ss >> m(i,j);
        }
      }

      return m;
    }

    Vector stringToVector(const std::string& str, unsigned length)
    {
      return stringToMatrix(str,length,1);
    }

    Vector stringToVector(const std::string& str)
    {
      Vector v;
      std::stringstream ss;
      ss << str;

      std::vector<double> doublecontainer;
      double component;
      bool readingVector = true;
      while (readingVector)
      {
        ss>> component;

        if (ss.fail())
        {
          readingVector=false;
        }
        else
        {
          doublecontainer.push_back(component);
        }
      }
      v.resize(doublecontainer.size());
      for (unsigned i=0 ; i<doublecontainer.size() ; ++i)
      {
        v(i)=doublecontainer[i];
      }

      return v;
    }
  }

}
