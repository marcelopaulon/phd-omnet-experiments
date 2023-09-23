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

network DADCA_ACK_n50_u2_a8000m2_2
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
        @display("i=misc/sensor;p=692.21,7999.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1483.32,7778.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1835.36,7788.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2766.88,7957.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4169.27,8000.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4827.73,7997.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=34.89,7666.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=708.09,7622.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1123.24,7688.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2446.53,7745.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3242.09,7590.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3849.52,7567.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4530.35,7601.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4884.33,7616.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5357.86,7631.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5996.86,7593.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=430.55,7264.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1368.28,7377.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1817.10,7362.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2238.07,7447.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2775.58,7383.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5558.96,7316.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=98.92,7047.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=791.24,7119.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3305.00,7216.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5006.23,7244.20");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1453.25,6958.40");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1927.18,6882.17");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2526.69,6989.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2968.88,6990.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3266.19,6779.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3908.73,6933.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4614.64,7003.39");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4909.35,6783.51");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5267.37,6957.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5715.59,6990.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=350.06,6599.91");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1353.87,6516.68");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4288.93,6701.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=10.24,6405.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=741.24,6461.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1714.56,6334.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2284.66,6482.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2623.32,6340.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3802.46,6342.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4468.80,6302.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4934.27,6325.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=307.26,6206.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=915.82,6041.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1314.19,6167.18");
    }

}

