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

network DADCA_ACK_n50_u8_a8000m2_5
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
        @display("i=misc/sensor;p=343.24,7957.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=745.79,7988.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1129.89,7972.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2016.24,7841.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2453.65,7923.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3026.85,7967.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3728.86,7822.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4115.50,7756.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5003.82,7950.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5397.67,7766.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=229.11,7584.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=644.50,7648.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1670.66,7745.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2853.97,7570.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3348.14,7741.79");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4710.55,7517.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5947.70,7548.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=492.25,7329.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1049.30,7412.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1848.11,7418.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2317.33,7371.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3696.33,7408.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4323.96,7458.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5192.04,7458.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2844.25,7119.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3219.50,7116.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5814.40,7221.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=309.03,6858.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=677.06,6826.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1171.85,6766.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1514.51,6877.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2243.07,7000.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3666.32,6950.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4077.34,6854.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4489.47,6842.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4858.95,6983.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5454.65,6832.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=475.85,6529.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=874.15,6530.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1727.51,6582.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2808.19,6609.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3258.13,6683.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5729.55,6533.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=131.76,6326.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2442.84,6400.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3566.62,6469.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4334.98,6362.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4798.95,6364.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5251.17,6468.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=973.56,6107.96");
    }

}

