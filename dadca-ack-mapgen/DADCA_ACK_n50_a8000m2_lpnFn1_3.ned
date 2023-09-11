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
        @display("i=misc/sensor,p=69.78,184.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=0.81,133.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=3.11,239.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=152.89,162.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=46.61,39.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=151.27,287.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=216.52,262.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=35.60,324.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=75.45,407.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=168.64,427.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=224.87,389.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=303.20,427.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=328.28,483.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=394.23,394.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=402.89,473.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=318.85,349.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=420.95,541.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=507.27,436.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=575.57,392.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=497.61,373.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=658.74,360.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=739.62,316.73");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=744.87,219.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=808.63,208.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=740.66,157.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=680.89,205.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=862.79,176.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=921.54,90.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=960.81,172.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=974.49,255.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=1040.63,297.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1122.06,325.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1080.88,411.76");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1160.52,411.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1058.60,507.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1118.79,549.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1213.39,506.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1224.84,577.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1302.11,540.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1359.56,459.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1376.23,379.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1271.39,636.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1166.52,609.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1296.91,397.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1307.80,331.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1242.42,271.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1173.97,291.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1027.79,454.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=967.88,379.94");
    }

}

