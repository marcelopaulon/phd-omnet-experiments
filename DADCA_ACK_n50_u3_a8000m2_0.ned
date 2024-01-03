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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=359.11,7786.53");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1220.31,7810.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1554.36,7954.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1940.56,7754.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2612.25,7991.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3807.08,7788.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4777.75,7903.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=897.82,7639.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2844.71,7635.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3278.49,7551.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4414.57,7597.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5081.83,7562.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5566.29,7731.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=150.52,7317.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=629.15,7262.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1797.25,7267.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2256.83,7423.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3040.81,7261.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4010.69,7488.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1119.06,7019.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2571.27,7146.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3664.68,7077.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4595.12,7120.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5294.99,7097.96");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5648.95,7232.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=142.16,6826.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2264.97,6970.35");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3343.16,6859.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4315.89,6839.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1189.89,6595.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1941.98,6668.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2785.76,6645.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3947.98,6537.32");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4864.08,6601.03");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5360.83,6617.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5766.36,6707.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=135.10,6334.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=644.95,6363.20");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=991.75,6304.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5653.54,6361.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1820.40,6073.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2338.70,6215.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3160.23,6234.04");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3640.51,6248.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4292.85,6264.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4971.57,6175.06");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5982.32,6223.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=522.42,5833.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1462.86,6011.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2767.51,5970.39");
    }

}

