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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=169.43,38.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=244.19,58.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=160.10,167.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=78.03,164.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=62.29,258.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=71.60,344.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=67.30,413.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=112.39,453.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=164.77,401.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=180.45,341.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=111.74,293.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=225.43,258.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=261.94,210.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=296.93,288.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=227.08,146.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=34.07,541.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=65.67,619.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=8.06,650.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=146.29,707.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=199.85,742.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=127.74,774.04");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=198.95,809.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=190.73,894.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=286.68,889.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=369.42,836.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=447.96,831.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=459.71,734.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=441.43,909.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=505.69,905.89");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=553.32,826.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=629.88,879.46");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=591.46,936.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=655.82,1051.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=672.82,1119.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=766.68,1136.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=889.10,1131.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=946.79,1173.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=942.16,1083.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=850.37,1051.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=776.20,1064.83");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=905.18,1001.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=967.59,992.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=907.53,920.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1049.31,948.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1030.28,1032.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1019.90,894.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1103.66,1032.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1165.20,1048.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1213.80,1005.07");
    }

}

