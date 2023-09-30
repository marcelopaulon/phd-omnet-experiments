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

network DADCA_ACK_n50_u16_a8000m2_9
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
        @display("i=misc/sensor;p=268.00,7994.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=772.02,7839.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1737.30,7886.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2248.52,7807.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2946.78,7847.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3439.43,7903.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4112.35,7928.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=50.79,7577.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=399.95,7529.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1341.99,7677.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3817.80,7541.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5030.74,7553.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5687.88,7562.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1029.88,7464.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2338.26,7395.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3478.53,7453.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4051.91,7265.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4439.33,7485.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5429.78,7287.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5967.36,7267.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=264.34,7186.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=634.45,7090.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1115.13,7023.69");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1731.84,7226.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2704.99,7038.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3536.13,7033.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4524.67,7073.07");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4900.98,7160.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1532.57,6818.60");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2147.02,6792.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3913.17,6903.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5692.06,6897.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=86.47,6571.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=734.57,6577.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1836.14,6543.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2505.52,6749.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2968.11,6699.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4308.42,6617.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4693.04,6670.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5067.56,6671.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1341.82,6509.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2271.94,6306.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2805.08,6309.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3494.75,6373.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3923.73,6292.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5270.01,6370.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5635.95,6451.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5971.79,6337.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=531.54,6105.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1112.21,6196.89");
    }

}

