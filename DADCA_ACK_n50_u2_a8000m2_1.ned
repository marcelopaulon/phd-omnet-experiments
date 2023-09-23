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

network DADCA_ACK_n50_u2_a8000m2_1
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
        @display("i=misc/sensor;p=515.09,7789.14");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=912.51,7989.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1385.52,7982.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1949.66,7760.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4396.27,7772.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5123.80,7753.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5522.28,7943.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1473.45,7552.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2373.55,7720.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2965.36,7714.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5809.87,7588.57");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=345.60,7400.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=927.31,7481.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1225.89,7287.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1855.05,7383.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2510.69,7354.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3979.35,7437.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4584.38,7317.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5166.11,7336.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5547.09,7319.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=617.01,7096.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2990.77,7047.99");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3608.35,7185.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4250.63,7074.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=940.51,6953.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1667.78,7007.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2195.48,7003.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3363.98,6884.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3851.12,6901.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4796.32,6895.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5268.49,6889.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=350.18,6594.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=914.99,6593.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1380.81,6700.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2729.70,6717.02");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3536.91,6541.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4465.67,6699.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1909.64,6490.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2310.45,6362.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3173.52,6341.60");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3967.72,6354.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4914.62,6433.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5741.69,6381.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=14.09,6054.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=821.50,6227.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1427.58,6089.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2860.82,6146.74");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3562.00,6041.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4593.34,6244.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5376.94,6213.73");
    }

}

