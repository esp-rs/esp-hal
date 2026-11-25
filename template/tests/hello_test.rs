//%if option("embassy")
//%include ".template/partials/hello_test_async.rs"
//%else
//%include ".template/partials/hello_test_blocking.rs"
//%endif
