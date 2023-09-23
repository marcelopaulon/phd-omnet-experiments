package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u4_a8000m2_5
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=1190.04,7782.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1710.81,7966.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3330.71,7800.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4763.93,7979.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5307.48,7878.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5667.04,7875.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=307.62,7674.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=715.52,7648.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2551.65,7745.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3761.55,7733.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5151.26,7505.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1044.24,7383.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1425.38,7376.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1793.74,7456.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2850.98,7481.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4103.72,7332.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4574.20,7414.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5756.53,7400.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=636.83,7054.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2191.45,7236.58");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2683.89,7049.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3164.41,7141.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3629.18,7102.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4807.60,7014.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5543.42,7023.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=246.98,6814.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1723.97,6961.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2219.48,6882.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4150.84,6858.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5217.42,6884.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=939.26,6539.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1287.54,6741.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4427.55,6610.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5682.76,6691.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=34.46,6496.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=521.32,6364.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1279.09,6378.76");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1861.84,6335.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2558.91,6364.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3076.92,6496.90");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3548.99,6495.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3982.92,6461.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4732.99,6392.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5972.18,6413.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=54.91,6047.16");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=829.68,6181.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3039.53,6036.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5396.84,6195.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5996.75,6028.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=491.38,5916.09");
    }

}

