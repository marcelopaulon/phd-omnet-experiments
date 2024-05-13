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

network DADCA_ACK_n50_u3_a8000m2_9
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
        @display("i=misc/sensor;p=399.41,7850.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2398.33,7893.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3347.86,7844.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3758.73,7980.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4441.14,7990.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4865.19,7823.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=805.10,7636.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1684.92,7531.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2147.80,7539.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2957.20,7699.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3597.40,7508.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4010.10,7589.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5241.29,7561.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3.39,7405.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=487.46,7277.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2673.52,7485.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2992.69,7333.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4612.07,7309.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4996.51,7305.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5852.71,7473.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1109.59,7208.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1780.86,7086.99");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2318.44,7166.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4074.01,7016.71");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5409.07,7191.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=10.06,6816.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1243.46,6830.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2752.10,6996.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3203.19,6994.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3640.15,6923.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4633.50,6901.74");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5044.88,6919.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5759.11,6942.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=519.18,6727.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=896.14,6655.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2091.17,6522.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3720.73,6539.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4669.54,6551.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5600.92,6605.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=97.08,6371.74");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=771.05,6303.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2601.79,6387.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2936.64,6514.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3313.43,6410.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4074.21,6489.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5314.85,6390.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5947.17,6364.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=415.30,6070.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1286.19,6260.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1707.00,6188.74");
    }

}

