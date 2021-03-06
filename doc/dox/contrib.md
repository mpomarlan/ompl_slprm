# External Contributions

We welcome external contributions. If you would like us to list your contribution on our [page](thirdparty.html), please [contact us](contact.html). If you would like to include your code in OMPL, please read on. We want to make the process of contributing code as easy as possible. To this end we came up with the following guidelines:

1. Contributions are each stored in a separate directory in \c ompl/src/ompl/contrib. The contributors can choose a descriptive name for their directory such as their group name or the name of their algorithm. We have created a sample contribution directory in \c ompl/src/ompl/contrib/sample_contrib that you can use for your contribution. Below, we will describe the expected structure of a contribution's directory in more detail.
2. Within the contribution’s directory there are the following files / directories:
   
   - __CMakeLists.txt__ [required], a file that tells the CMake program how to build your code. Use the one in the sample_contrib directory as a template.
   - __src__ [required], a directory that contains C++ implementation files. The corresponding header files should be inside the sample_contrib directory itself.
   - __tests__ [required], a directory that contains testing code that demonstrates that essential bits of code are running correctly
   - __README.txt/README.md__ [required], a high-level description of the functionality of the contribution(s). This can include primary citations.
   - __doc__ [optional], additional documentation, ideally in MarkDown format (analogous to the high-level OMPL documentation). Images or (short) videos are also welcome.

   All source code files must have a BSD license at the top of each file plus the names of the authors. Also, the code should contain doxygen-style documentation, so that the API documentation can be automatically generated and included in the OMPL web site. Finally, the code should be formatted according to the [OMPL style guide](styleGuide.html).
3. We will add a description to the [contributions page](thirdparty.html) (accessible from the “Community” menu at the top of the page) based on the content of the contribution's README.txt. It will include a proper acknowledgements of the authors. There will also be a link to the more detailed documentation generated from the files in the doc directory.
4. We will add the appropriate CMake code to \c ompl/src/ompl/CMakeLists to compile the code in a contribution’s src directory. Most likely, all that is required is the addition of just one line:

        add_subdirectory(contrib/sample_contrib)

5. It is possible that some people write code that simply uses OMPL as a low-level dependency. In that case, bundling that code with OMPL might not make sense, but we can still list this as contribution with links to an external web site if it is likely that users of OMPL are also likely to be interested in this code.
6. The mechanics of contributions are handled in one of two ways:

    - Mercurial [preferred]: contributors clone the OMPL repository on bitbucket.org like so:
            
          $ hg clone https://bitbucket.org/ompl/ompl
            
      Contributors should then create their own branch, add their files in the appropriate places, and either send us a “pull” request (if the repository is accessible to us) or email us a bundle (see http://www.selenic.com/mercurial/hg.1.html#bundle).
    - tar balls / zip files / etc.: contributors just email us their files and we put them in the appropriate place.
7. In the unlikely event that the submitted code is of poor quality, we will try to suggest the changes necessary to include the contribution in OMPL. If the contributors cannot do this themselves, we reserve the right to reject the contribution.
