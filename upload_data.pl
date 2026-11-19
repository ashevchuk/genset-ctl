#!/usr/bin/perl
use strict;
use warnings;
use FindBin qw($RealBin);
use File::Find;
use File::Basename;
use LWP::UserAgent;
use HTTP::Request::Common qw(POST);
use POSIX qw(floor);

# Run relative to this script's own location, not the caller's CWD — so
# `./upload_data.pl` works the same whether invoked from the repo root or
# from anywhere else, and the repo can live at any path.
chdir $RealBin or die "Cannot chdir to $RealBin: $!\n";

# ── Config ──────────────────────────────────────────────────────────────────
my $DATA_DIR  = "data";
my $HOST_FILE = "host";
my $PORT      = 80;

# Files to skip (backups, sources that have .gz counterpart, etc.)
my @SKIP_EXT = qw(.bak .map);

# ── Read host ────────────────────────────────────────────────────────────────
# Optional CLI override (./upload_data.pl 10.100.100.97) so this can target
# the test rig without touching the repo-root `host` file, which several
# other scripts (burn.sh, decode_crash.sh's default) also read — same
# reasoning as flash.sh's own [port] argument. Defaults to `host` exactly as
# before when no argument is given.
my $host;
if (@ARGV && $ARGV[0] ne '') {
    $host = $ARGV[0];
} else {
    open(my $fh, '<', $HOST_FILE) or die "Cannot read $HOST_FILE: $!\n";
    $host = <$fh>; close $fh;
}
$host =~ s/\s+//g;
my $base_url = "http://$host:$PORT";
print "Target: $base_url\n\n";

# ── Collect dirs and files ───────────────────────────────────────────────────
my @dirs;
my @files;

find({
    wanted => sub {
        my $abs  = $File::Find::name;
        my $rel  = $abs;
        $rel =~ s{^\Q$DATA_DIR\E/?}{};
        return unless $rel;           # skip root itself

        if (-d $abs) {
            push @dirs, $rel;
        } else {
            # Check skip list
            my (undef, undef, $ext) = fileparse($abs, qr/\.[^.]*/);
            return if grep { $_ eq lc($ext) } @SKIP_EXT;
            push @files, { abs => $abs, rel => $rel };
        }
    },
    no_chdir => 1,
}, $DATA_DIR);

# Sort dirs by depth so parents are created first
@dirs = sort { ($a =~ tr{/}{}) <=> ($b =~ tr{/}{}) } @dirs;

# ── HTTP client ──────────────────────────────────────────────────────────────
my $ua = LWP::UserAgent->new(timeout => 30);

sub check_response {
    my ($res, $label) = @_;
    if ($res->is_success) {
        print "  OK\n";
    } else {
        print "  FAIL: " . $res->status_line . "\n";
        warn "Failed: $label\n";
    }
}

# ── 1. Create directories ────────────────────────────────────────────────────
print "=== Creating directories (" . scalar(@dirs) . ") ===\n";
for my $dir (@dirs) {
    my $dn     = dirname($dir);
    my $parent = ($dn eq '.') ? "/" : "/$dn/";
    $parent = "/" if $parent eq "//";

    my $name = basename($dir);

    print "  mkdir  /$dir  (path=$parent name=$name) ... ";

    my $req = POST "$base_url/api/mkdir",
        Content_Type => 'application/x-www-form-urlencoded',
        Content      => "path=$parent&name=$name";

    check_response($ua->request($req), "mkdir /$dir");
}

# ── 2. Upload files ──────────────────────────────────────────────────────────
print "\n=== Uploading files (" . scalar(@files) . ") ===\n";
my $n = 0;
for my $f (@files) {
    $n++;
    my $rdn        = dirname($f->{rel});
    my $remote_dir = ($rdn eq '.') ? "/" : "/$rdn/";
    $remote_dir = "/" if $remote_dir eq "//";

    my $fname = basename($f->{abs});
    my $size  = -s $f->{abs};
    printf "  [%d/%d] %-40s -> %s ... ",
        $n, scalar(@files), $f->{rel}, $remote_dir;

    my $req = POST "$base_url/api/upload?path=$remote_dir",
        Content_Type => 'multipart/form-data',
        Content      => [ file => [ $f->{abs}, $fname ] ];

    check_response($ua->request($req), "upload " . $f->{rel});
}

print "\nDone.\n";
