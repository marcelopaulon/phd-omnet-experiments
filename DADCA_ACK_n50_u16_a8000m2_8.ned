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

network DADCA_ACK_n50_u16_a8000m2_8
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
        @display("i=misc/sensor;p=963.67,7905.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1515.84,7968.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2036.76,7891.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3320.21,7805.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1566.67,7610.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2355.26,7679.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=630.80,7402.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1211.47,7335.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1991.18,7504.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2435.08,7281.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2797.79,7499.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4331.52,7342.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5561.25,7356.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=817.15,7094.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3272.41,7224.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3720.98,7089.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5017.79,7183.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5957.21,7199.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=361.38,7000.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1319.60,6910.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1721.49,6969.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2140.28,6832.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2746.43,6957.59");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4397.68,6808.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4726.20,6932.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5789.32,6877.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=950.27,6753.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3972.64,6627.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4858.49,6539.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5340.70,6645.31");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5950.08,6540.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=701.65,6415.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1468.14,6476.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1819.26,6404.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2541.26,6384.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3426.29,6480.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4518.39,6357.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1334.26,6126.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2200.33,6145.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3141.82,6143.74");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3556.33,6082.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4123.05,6160.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5143.05,6056.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5576.69,6248.89");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=311.87,5999.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=914.85,5981.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2576.86,5958.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3247.63,5780.12");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5913.02,5895.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=369.76,5532.41");
    }

}

