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

network DADCA_ACK_n50_u2_a8000m2_3
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
        @display("i=misc/sensor;p=671.51,7970.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1266.01,7991.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2562.03,7844.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3221.39,7902.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5189.31,7957.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5508.96,7765.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=605.12,7543.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1095.08,7602.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1866.54,7684.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3383.41,7541.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3940.37,7543.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4401.11,7743.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4959.19,7581.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5942.59,7709.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1406.42,7418.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2363.55,7304.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2916.12,7383.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5389.04,7390.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=302.13,7095.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=933.98,7255.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1784.97,7123.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3541.15,7220.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4182.34,7219.33");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4907.18,7107.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5756.62,7107.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1182.40,6984.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2272.88,6841.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3054.30,6825.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3780.51,6846.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4271.15,6868.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4688.18,6796.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=368.63,6608.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1728.26,6683.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3351.34,6592.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5286.20,6605.70");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1115.57,6384.06");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1779.62,6329.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2424.45,6486.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2877.65,6414.71");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3783.55,6268.92");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4331.84,6376.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4833.30,6474.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5937.58,6458.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=285.43,6137.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=689.29,6080.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1364.76,6119.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5295.04,6150.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5694.11,6112.38");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1233.20,5777.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2015.39,5942.68");
    }

}

