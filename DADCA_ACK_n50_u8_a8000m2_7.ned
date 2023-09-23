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

network DADCA_ACK_n50_u8_a8000m2_7
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
        @display("i=misc/sensor;p=44.22,7838.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=588.01,7844.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1084.86,7791.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1507.27,7768.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2221.60,7850.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3349.63,7774.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3997.31,7910.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4344.25,7958.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5602.27,7822.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1946.49,7505.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2794.80,7721.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3942.80,7530.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4426.59,7599.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4963.95,7734.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=336.31,7274.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=741.25,7302.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1257.12,7266.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3317.40,7309.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5990.67,7410.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2498.82,7225.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2853.12,7221.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4605.25,7027.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5001.52,7072.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5551.19,7197.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1788.99,6964.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2235.67,6965.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2950.74,6847.10");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3628.09,6973.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3980.90,6989.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5317.86,6911.81");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=278.65,6541.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=619.23,6694.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1263.48,6539.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1664.71,6583.58");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3616.78,6549.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3963.65,6609.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4917.30,6535.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5776.22,6610.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2282.35,6504.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3003.63,6451.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4330.44,6411.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5537.68,6296.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=173.01,6086.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=833.41,6219.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1995.60,6042.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2603.22,6267.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2986.03,6100.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3461.28,6175.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4374.55,6025.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1188.06,5938.57");
    }

}

