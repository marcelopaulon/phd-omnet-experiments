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

network DADCA_ACK_n50_u3_a8000m2_3
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
        @display("i=misc/sensor;p=440.84,7983.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=915.96,7888.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1768.98,7843.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2255.23,7753.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3091.08,7978.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3429.65,7857.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4578.36,7861.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5353.83,7905.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5836.91,7988.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=812.26,7518.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1216.84,7547.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4147.48,7733.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5991.48,7633.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=146.03,7457.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1695.55,7439.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2488.99,7333.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2842.73,7482.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3266.60,7499.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3821.81,7347.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4364.04,7280.90");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4795.39,7501.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5157.25,7485.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5563.49,7432.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=483.43,7254.19");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1087.44,7117.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2077.27,7050.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3511.95,7082.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5249.78,7045.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5889.50,7095.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=725.27,6883.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1405.08,6799.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1771.73,6795.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3001.47,6915.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4581.94,6872.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=141.49,6731.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1046.07,6554.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2349.84,6759.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3484.92,6592.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4031.16,6757.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5840.25,6690.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=327.15,6405.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1707.14,6320.69");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2172.95,6419.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2972.29,6441.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4481.12,6483.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5070.87,6377.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=693.41,6149.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1308.22,6197.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2573.87,6054.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3553.29,6150.21");
    }

}

