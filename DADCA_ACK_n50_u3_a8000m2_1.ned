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

network DADCA_ACK_n50_u3_a8000m2_1
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
        @display("i=misc/sensor;p=932.82,7936.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2160.28,7898.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3978.75,7974.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5838.65,7810.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=165.44,7691.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=710.35,7523.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1142.89,7506.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2472.20,7709.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2960.96,7518.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3493.81,7747.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4111.98,7556.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4790.50,7591.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5485.10,7635.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1662.83,7378.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2227.70,7323.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3600.70,7405.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4456.06,7274.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=16.11,7129.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=945.49,7138.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1813.09,7034.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2588.31,7193.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3086.37,7027.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5127.52,7185.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5902.53,7156.09");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=462.26,6972.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1257.30,6945.05");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2218.18,6941.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3587.92,6987.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4125.10,6768.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=257.66,6520.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=652.25,6548.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1484.70,6580.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1877.61,6570.07");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2488.39,6613.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3516.97,6586.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4753.26,6644.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5516.70,6642.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1202.40,6342.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2221.08,6385.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3045.87,6355.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3833.15,6347.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5082.68,6411.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=657.33,6088.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1788.22,6129.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2601.46,6090.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3483.99,6092.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4136.88,6091.39");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4550.02,6096.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1260.55,5895.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1978.31,5802.04");
    }

}

