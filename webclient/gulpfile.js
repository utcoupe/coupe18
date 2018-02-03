const gulp = require('gulp'),
  babel = require('gulp-babel'),
  concat = require('gulp-concat'),
  eslint = require('gulp-eslint');

function swallowError (error) {

  // If you want details of the error in the console
  console.log(error.toString())

  this.emit('end')
}

// Concat all js files into roscc.js
gulp.task('js', function () {
  return gulp.src(['app/app.js', 'app/**/*.js'])
    .on('error', swallowError)
    .pipe(babel({ presets: ['es2015'] }))
    .on('error', swallowError)
    .pipe(concat('roscc.js'))
    .pipe(gulp.dest('assets/js/'));
});

gulp.task('js-vendor', function () {
  return gulp.src([
    'node_modules/underscore/underscore.js',
    'node_modules/angular/angular.js',
    'node_modules/angular-animate/angular-animate.js',
    'node_modules/angular-route/angular-route.js',
    'node_modules/angular-local-storage/dist/angular-local-storage.js',
    'node_modules/angular-ui-bootstrap/dist/ui-bootstrap-tpls.js',
    'node_modules/eventemitter2/lib/eventemitter2.js',
    'node_modules/raphael/raphael.js',
    'node_modules/jquery/dist/jquery.js',
    'node_modules/three/build/three.js',
    'node_modules/chart.js/dist/Chart.js',
    'node_modules/angular-chart.js/dist/angular-chart.js'

    //'node_modules/three-collada-loader/index.js'
  ])
    .pipe(concat('vendor.js'))
    .pipe(gulp.dest('assets/js/'));
});

// Lint javascript based on airbnb ES5 linter and angular code style guide
gulp.task('js-lint', function() {
  return gulp.src(['app/**/*.js'])
    .pipe(eslint()) // use .eslintrc.json file for rules
    .pipe(eslint.format())
//    /*.pipe(eslint.failAfterError())*/;
});

// Changes will be detected automatically
gulp.task('watch', function () {
  gulp.watch('app/**/*.js', ['js']);
});

gulp.task('default', [/*'js-lint', */'js-vendor', 'js', 'watch']);
