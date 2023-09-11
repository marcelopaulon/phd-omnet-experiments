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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=27.94,155.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=153.76,186.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=243.86,182.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=212.98,239.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=130.31,282.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=166.06,93.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=152.62,16.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=56.50,48.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=252.67,121.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=282.47,39.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=356.40,79.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=428.28,144.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=431.40,74.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=515.85,111.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=580.52,69.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=633.68,10.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=708.22,37.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=695.22,173.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=751.46,253.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=718.09,331.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=802.84,361.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=878.85,346.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=912.84,272.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=856.33,240.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=658.79,113.73");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=815.39,175.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=917.46,405.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=859.85,441.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=881.03,499.15");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=970.25,463.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=875.90,559.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=949.90,341.61");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1020.11,289.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1037.97,373.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1088.84,409.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1031.30,493.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1045.95,580.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1091.09,516.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1154.69,544.78");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1183.33,636.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1157.47,707.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1198.03,759.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1150.72,817.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1234.29,820.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1331.12,801.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1286.64,870.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1283.98,737.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1443.54,851.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1496.26,886.90");
    }

}

