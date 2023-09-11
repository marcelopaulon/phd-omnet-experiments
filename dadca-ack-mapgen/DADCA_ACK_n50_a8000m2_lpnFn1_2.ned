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

network DADCA_ACK_n50_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=34.77,89.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=59.48,158.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=40.37,11.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=170.91,123.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=233.56,174.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=290.18,242.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=253.20,294.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=247.81,55.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=323.28,4.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=146.20,40.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=319.03,124.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=143.47,221.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=333.95,338.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=421.59,361.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=475.06,399.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=431.15,447.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=364.72,508.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=426.68,557.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=493.52,564.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=433.66,621.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=389.75,699.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=365.49,775.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=311.95,834.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=255.44,770.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=301.74,713.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=386.80,868.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=445.64,841.11");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=346.99,920.18");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=275.42,967.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=326.24,1020.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=393.72,979.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=369.11,1064.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=247.67,1095.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=210.78,1041.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=208.55,1141.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=183.93,1211.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=121.75,1281.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=35.86,1276.88");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=16.64,1339.12");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=76.50,1212.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=147.05,1145.14");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=120.55,1359.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=95.43,1463.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=13.46,1445.62");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=32.18,1514.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=86.76,1576.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=121.91,1641.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=210.68,1543.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=247.94,1612.56");
    }

}

