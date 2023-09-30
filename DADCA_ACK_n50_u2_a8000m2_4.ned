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

network DADCA_ACK_n50_u2_a8000m2_4
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
        @display("i=misc/sensor;p=1929.96,7811.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2527.23,7779.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3772.16,7996.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4720.25,7937.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5448.72,7933.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5964.40,7843.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=26.01,7595.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1030.41,7740.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3039.37,7685.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3600.79,7588.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4085.10,7725.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5066.03,7723.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5488.85,7569.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1414.06,7487.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2546.20,7410.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4514.97,7380.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=79.84,7137.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=465.34,7023.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1049.69,7231.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1814.31,7171.54");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2168.98,7178.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3123.29,7171.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3918.13,7189.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4843.51,7065.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5688.86,7184.04");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2481.92,6912.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3607.29,6842.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4500.96,6971.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5313.96,6929.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=9.56,6727.15");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=809.71,6690.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1431.62,6683.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1850.35,6541.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2744.52,6640.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3141.66,6568.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4485.69,6600.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5000.81,6626.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5704.39,6645.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=455.89,6365.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1117.33,6509.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2307.37,6394.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3812.49,6510.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4172.53,6377.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5313.06,6425.98");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1512.77,6146.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1979.45,6122.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3024.18,6184.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3526.33,6265.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4563.90,6220.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5887.69,6143.82");
    }

}

