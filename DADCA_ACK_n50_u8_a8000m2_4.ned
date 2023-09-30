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

network DADCA_ACK_n50_u8_a8000m2_4
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
        @display("i=misc/sensor;p=1218.09,7804.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2453.39,7850.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3335.25,7922.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4519.31,7913.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4868.28,7992.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5392.44,7952.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=579.68,7678.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=945.52,7574.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1941.57,7616.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3588.62,7674.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4192.95,7643.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5059.93,7665.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=329.53,7412.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1309.11,7459.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3011.86,7376.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3764.49,7296.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4687.77,7493.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5772.06,7456.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=523.66,7120.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1634.98,7055.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2072.57,7106.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2528.81,7151.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4257.31,7014.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4836.78,7049.73");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=153.13,6988.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1173.20,6968.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3212.01,6839.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3662.75,6914.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5452.49,6871.37");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=599.81,6703.24");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1018.94,6628.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1673.73,6688.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2301.20,6660.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=21.23,6470.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2858.49,6510.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3122.23,6275.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3506.05,6448.35");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4146.93,6507.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4777.32,6447.86");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5435.93,6492.38");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5980.29,6376.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=423.27,6078.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=744.30,6241.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1469.78,6123.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2019.59,6187.18");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2525.00,6027.65");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5427.04,6044.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=812.21,5835.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2954.61,5858.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3544.57,5906.14");
    }

}

