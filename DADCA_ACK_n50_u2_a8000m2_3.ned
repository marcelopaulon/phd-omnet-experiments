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

network DADCA_ACK_n50_u2_a8000m2_3
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
        @display("i=misc/sensor;p=542.26,7812.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1131.67,7791.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1520.91,7966.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2118.98,7782.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2678.43,7895.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3164.10,7813.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4273.40,7983.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=120.97,7703.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3514.95,7727.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4020.42,7641.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4948.23,7544.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5512.86,7617.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5960.83,7598.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=430.63,7274.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1658.90,7378.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2971.94,7439.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4491.21,7453.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1132.88,7250.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2193.10,7108.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3182.63,7124.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3529.68,7256.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4197.36,7135.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5354.80,7201.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5834.83,7215.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=65.00,6959.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=648.20,6784.24");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1102.78,6900.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1553.47,6982.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2803.65,6975.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4565.31,6936.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4942.68,6932.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1743.69,6579.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2225.46,6645.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2597.32,6633.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3620.22,6747.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4053.32,6545.25");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5546.91,6622.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=187.15,6442.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3030.71,6503.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4618.41,6433.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5049.48,6435.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5898.39,6371.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=667.91,6259.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1060.54,6246.21");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1478.50,6216.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2186.41,6190.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2756.43,6210.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3663.16,6254.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4724.63,6088.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5199.96,6037.21");
    }

}

