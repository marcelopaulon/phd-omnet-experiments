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

network DADCA_ACK_n50_a8000m2_4
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
        @display("i=misc/sensor;p=1594.40,7874.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2139.16,7938.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4272.90,7896.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5276.43,7841.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=386.31,7677.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1057.15,7596.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1991.35,7611.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2550.84,7570.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3092.83,7583.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3504.60,7598.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4047.59,7513.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4679.79,7586.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5834.75,7669.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=130.87,7320.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1323.09,7286.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2161.78,7302.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4930.80,7309.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5576.75,7265.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=872.23,7162.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3424.55,7097.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3982.43,7092.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4342.21,7195.21");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5982.29,7115.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=477.40,6961.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1177.06,6939.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1569.01,6906.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1940.30,6809.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2925.63,6991.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4203.16,6771.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4596.66,6931.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5131.97,6927.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5731.57,6847.99");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=171.52,6559.23");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=719.33,6519.19");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1409.98,6588.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2395.83,6561.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3354.26,6693.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4881.58,6652.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1066.67,6462.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1852.96,6453.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2914.79,6331.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3702.71,6512.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4485.65,6504.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5769.96,6479.89");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=362.00,6176.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1464.65,6188.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2107.13,6182.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2533.56,6063.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3347.38,6097.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4145.32,6088.65");
    }

}

