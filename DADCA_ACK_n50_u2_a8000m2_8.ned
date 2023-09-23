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

network DADCA_ACK_n50_u2_a8000m2_8
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
        @display("i=misc/sensor;p=256.72,7800.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=617.46,7947.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2658.04,7955.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3302.87,7936.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3728.00,7776.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1423.79,7538.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2059.59,7683.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2454.50,7581.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3421.42,7513.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4176.74,7528.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5470.04,7568.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=394.34,7409.91");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=935.88,7340.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1753.97,7376.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3070.65,7416.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3784.39,7322.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=71.04,7037.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2116.36,7128.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2697.97,7216.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3125.51,7050.30");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5146.68,7159.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5773.18,7148.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=570.61,6987.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1079.59,6955.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1629.94,6994.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3596.90,6964.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4086.32,6946.23");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4470.82,6869.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5474.86,6934.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5920.36,6776.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=31.83,6671.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=424.82,6647.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1311.20,6673.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2228.22,6664.84");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2691.68,6609.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3126.35,6694.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4902.66,6682.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5441.28,6571.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=681.65,6343.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1375.25,6288.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2025.93,6357.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3745.67,6498.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4233.82,6325.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=71.38,6246.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2993.65,6234.74");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3529.74,6181.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4932.54,6203.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5523.93,6131.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5901.69,6193.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=26.81,5775.77");
    }

}

