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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=2849.91,7992.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3643.17,7811.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4044.47,7797.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4529.68,7897.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5149.84,7774.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=329.31,7724.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=751.23,7744.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1132.61,7728.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1534.42,7700.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2106.26,7590.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2475.00,7592.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2988.86,7618.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4806.78,7576.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=609.97,7270.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1648.14,7357.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3002.14,7266.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3506.66,7353.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3860.76,7420.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4466.96,7386.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5476.72,7452.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1153.83,7195.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2054.82,7194.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5009.87,7166.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2071.11,6800.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2484.59,6933.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2997.69,6895.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4079.02,6871.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4710.24,6920.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5903.54,6959.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=340.28,6708.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1025.60,6725.22");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1576.86,6737.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2679.62,6628.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3037.09,6545.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3633.00,6679.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5344.62,6630.39");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5725.68,6658.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=495.75,6319.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1463.32,6279.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4296.63,6500.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4672.94,6400.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5046.39,6440.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=885.20,6213.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2010.29,6225.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2590.12,6264.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3332.57,6233.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3795.14,6187.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4221.07,6123.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5074.27,6071.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5753.39,6224.20");
    }

}

