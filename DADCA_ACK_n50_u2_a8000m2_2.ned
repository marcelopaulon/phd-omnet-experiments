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

network DADCA_ACK_n50_u2_a8000m2_2
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
        @display("i=misc/sensor;p=175.09,7886.35");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1243.86,7843.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2895.89,7890.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4698.98,7824.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5953.16,7812.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=606.41,7640.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2092.65,7668.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2450.43,7621.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3909.02,7673.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4431.37,7577.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2973.95,7291.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5068.92,7266.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5448.92,7394.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=844.45,7124.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1411.17,7211.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2059.04,7136.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2546.27,7178.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3246.47,7046.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3792.32,7243.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4326.53,7066.95");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4716.34,7124.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5961.77,7191.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=609.12,6792.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1637.26,6838.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2122.68,6777.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3640.27,6812.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5339.92,6855.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1106.74,6671.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2442.72,6618.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2965.60,6660.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4012.09,6646.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4852.53,6678.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=862.45,6308.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1501.24,6288.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2058.85,6432.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3089.14,6310.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3466.69,6491.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4286.49,6298.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5165.56,6460.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5929.98,6495.39");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=327.51,6114.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1914.17,6040.52");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2565.93,6094.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4011.43,6079.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4815.33,6056.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=646.36,5959.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2222.34,5779.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3087.85,5795.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3503.04,5787.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5529.12,5969.92");
    }

}

