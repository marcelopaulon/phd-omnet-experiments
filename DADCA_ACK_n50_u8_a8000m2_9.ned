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

network DADCA_ACK_n50_u8_a8000m2_9
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
        @display("i=misc/sensor;p=645.18,7930.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1133.82,7908.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2937.77,7945.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3276.83,7800.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4034.12,7901.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5944.39,7752.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1607.61,7679.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2550.53,7643.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3599.06,7517.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4561.98,7722.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5053.17,7578.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5504.37,7730.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=130.11,7497.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=685.80,7435.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1258.30,7427.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3199.57,7385.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3957.61,7458.51");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4458.41,7374.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5409.56,7372.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=329.99,7144.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1939.25,7032.88");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=160.26,6796.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=723.47,6766.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1463.10,7000.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2382.42,6811.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2743.66,6789.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3306.51,6939.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3760.34,6893.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4115.58,6847.15");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4781.15,6960.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5627.85,6823.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1177.63,6614.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4810.92,6528.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5101.80,6747.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=303.69,6407.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=757.77,6351.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1666.80,6347.85");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2145.85,6423.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2958.35,6507.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3462.89,6310.85");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3984.13,6503.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4292.00,6332.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5219.94,6409.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5793.18,6453.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1295.91,6077.18");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2560.92,6082.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5955.07,6107.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=689.40,5832.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1811.82,5905.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3016.57,5867.21");
    }

}

