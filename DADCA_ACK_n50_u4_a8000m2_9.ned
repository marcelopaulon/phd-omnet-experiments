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

network DADCA_ACK_n50_u4_a8000m2_9
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
        @display("i=misc/sensor;p=25.58,7859.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=465.65,7806.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1142.18,7877.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1982.00,7911.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2621.09,7912.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3196.16,7908.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3705.85,7959.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4380.01,7951.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5126.50,7974.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5957.75,7908.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1259.75,7516.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2135.06,7532.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3475.01,7541.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4010.88,7600.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5509.10,7613.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=70.89,7304.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=612.05,7300.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1610.04,7477.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3094.79,7313.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4389.44,7359.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4873.06,7469.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5855.64,7411.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1043.55,7223.90");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1805.28,7036.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2526.35,7209.33");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5020.67,7076.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5413.01,7020.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=553.21,6946.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2996.77,6972.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3611.20,6862.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4623.45,6921.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5813.12,6842.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1043.69,6552.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1420.67,6516.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2044.01,6713.48");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2487.59,6727.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3279.28,6577.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4097.56,6602.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5385.32,6596.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=30.49,6434.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=571.23,6507.90");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1955.98,6313.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3017.97,6322.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3696.27,6397.86");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4649.59,6492.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5015.16,6413.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=357.19,6161.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1161.86,6069.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2264.93,6113.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4256.53,6131.90");
    }

}

