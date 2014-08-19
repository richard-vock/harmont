template <typename Scalar, GLenum Target>
template <template <typename, typename> class C, template <typename> class A>
inline typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const C<Scalar, A<Scalar>>& sequence, GLenum usage) {
    return vertex_buffer<Scalar, Target>::from_data(sequence.data(), sequence.size(), usage);
}

template <typename Scalar, GLenum Target>
template <template <typename, typename> class C, typename S, template <typename> class A>
inline typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const C<S, A<S>>& sequence, GLenum usage) {
    C<Scalar, A<Scalar>> cast_copy(sequence.begin(), sequence.end());
    return vertex_buffer<Scalar, Target>::from_data(cast_copy.data(), cast_copy.size(), usage);
}

template <typename Scalar, GLenum Target>
template <int Dim, template <typename, typename> class C, template <typename> class A>
inline typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const C<eigen_vector<Scalar, Dim>, A<eigen_vector<Scalar, Dim>>>& vector_sequence, GLenum usage) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix(vertex_sequence.size(), Dim);
    for (uint32_t i = 0; i < vector_sequence.size(); ++i) {
        matrix.row(i) = vector_sequence[i].transpose();
    }
    return vertex_buffer<Scalar, Target>::from_data(matrix, usage);
}

template <typename Scalar, GLenum Target>
template <int Dim, template <typename, typename> class C, typename S, template <typename> class A>
inline typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const C<eigen_vector<S, Dim>, A<eigen_vector<S, Dim>>>& vector_sequence, GLenum usage) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix(vertex_sequence.size(), Dim);
    for (uint32_t i = 0; i < vector_sequence.size(); ++i) {
        matrix.row(i) = vector_sequence[i].transpose().template cast<Scalar>();
    }
    return vertex_buffer<Scalar, Target>::from_data(matrix, usage);
}

template <typename Scalar, GLenum Target>
template <int Rows, int Cols, int Options>
inline typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix, GLenum usage) {
    if (Options & Eigen::RowMajor) {
        return vertex_buffer<Scalar, Target>::from_data(matrix.data(), matrix.rows() * matrix.cols(), usage);
    }
    int TransOptions = Options ^ Eigen::ColMajor;
    Eigen::Matrix<Scalar, Rows, Cols, TransOptions> data_transposed = matrix;
    return vertex_buffer<Scalar, Target>::from_data(data_transposed.data(), matrix.rows() * matrix.cols(), usage);
}

template <typename Scalar, GLenum Target>
template <typename S, int Rows, int Cols, int Options>
inline typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const Eigen::Matrix<S, Rows, Cols, Options>& matrix, GLenum usage) {
    return vertex_buffer<Scalar, Target>::from_data(matrix.template cast<Scalar>(), usage);
}

template <typename Scalar, GLenum Target>
template <template <typename, typename> class C, template <typename> class A>
inline void vertex_buffer<Scalar, Target>::set_data(const C<Scalar, A<Scalar>>& sequence) {
    set_data(sequence.data(), sequence.size());
}

template <typename Scalar, GLenum Target>
template <template <typename, typename> class C, typename S, template <typename> class A>
inline void vertex_buffer<Scalar, Target>::set_data(const C<S, A<S>>& sequence) {
    C<Scalar, A<Scalar>> cast_copy(sequence.begin(), sequence.end());
    set_data(cast_copy.data(), cast_copy.size());
}

template <typename Scalar, GLenum Target>
template <int Dim, template <typename, typename> class C, template <typename> class A>
inline void vertex_buffer<Scalar, Target>::set_data(const C<eigen_vector<Scalar, Dim>, A<eigen_vector<Scalar, Dim>>>& vector_sequence) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix(vertex_sequence.size(), Dim);
    for (uint32_t i = 0; i < vector_sequence.size(); ++i) {
        matrix.row(i) = vector_sequence[i].transpose();
    }
    set_data(matrix);
}

template <typename Scalar, GLenum Target>
template <int Dim, template <typename, typename> class C, typename S, template <typename> class A>
inline void vertex_buffer<Scalar, Target>::set_data(const C<eigen_vector<S, Dim>, A<eigen_vector<S, Dim>>>& vector_sequence) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> matrix(vertex_sequence.size(), Dim);
    for (uint32_t i = 0; i < vector_sequence.size(); ++i) {
        matrix.row(i) = vector_sequence[i].transpose().template cast<Scalar>();
    }
    set_data(matrix);
}

template <typename Scalar, GLenum Target>
template <int Rows, int Cols, int Options>
inline void vertex_buffer<Scalar, Target>::set_data(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix) {
    if (Options & Eigen::RowMajor) {
        return vertex_buffer<Scalar, Target>::from_data(matrix.data(), matrix.rows() * matrix.cols());
    }
    int TransOptions = Options ^ Eigen::ColMajor;
    Eigen::Matrix<Scalar, Rows, Cols, TransOptions> data_transposed = matrix;
    set_data(data_transposed.data(), matrix.rows() * matrix.cols());
}

template <typename Scalar, GLenum Target>
template <typename S, int Rows, int Cols, int Options>
inline void vertex_buffer<Scalar, Target>::set_data(const Eigen::Matrix<S, Rows, Cols, Options>& matrix) {
    return vertex_buffer<Scalar, Target>(matrix.template cast<Scalar>());
}

template <typename Scalar, GLenum Target>
template <template <typename, typename> class C, typename S, template <typename> class A>
inline void vertex_buffer<Scalar, Target>::get_data(C<S, A<S>>& sequence) {
    Scalar* data = new Scalar[element_count_];
    get_data(data);

    sequence.resize(element_count_);
    sequence.assign(data, data+element_count_);

    delete [] data;
}

template <typename Scalar, GLenum Target>
template <int Dim, template <typename, typename> class C, typename S, template <typename> class A>
inline void vertex_buffer<Scalar, Target>::get_data(C<eigen_vector<S, Dim>, A<eigen_vector<S, Dim>>>& vector_sequence) {
    Scalar* data = new Scalar[element_count_];
    get_data(data);

    uint32_t vec_count = element_count_ / static_cast<uint32_t>(Dim);
    vector_sequence.resize(vec_count);
    for (uint32_t i = 0; i < vec_count; ++i) {
        vector_sequence[i] = Eigen::Map<eigen_vector<S, Dim>>(data + i*Dim).template cast<S>();
    }

    delete [] data;
}

template <typename Scalar, GLenum Target>
template <typename S, int Rows, int Cols, int Options>
inline void vertex_buffer<Scalar, Target>::get_data(Eigen::Matrix<S, Rows, Cols, Options>& matrix) {
    Scalar* data = new Scalar[element_count_];
    get_data(data);

    matrix = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Options>>(data).template cast<S>();

    delete [] data;
}
