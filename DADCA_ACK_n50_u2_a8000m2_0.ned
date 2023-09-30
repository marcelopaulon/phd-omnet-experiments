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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=1123.26,7989.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1517.40,7941.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2398.15,7954.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2749.79,7990.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3484.14,7847.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4310.68,7968.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5066.42,7889.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5751.01,7909.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=352.51,7701.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=721.88,7659.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1986.70,7743.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2334.90,7516.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2995.46,7724.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3988.79,7557.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5442.18,7704.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1289.78,7444.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1624.11,7323.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2764.62,7382.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3324.74,7419.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3750.84,7297.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4564.66,7500.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5097.26,7364.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5459.11,7319.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=131.00,7221.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=762.70,7136.17");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2239.45,7155.65");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5842.72,7240.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1153.42,6965.02");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1772.45,6923.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2162.28,6794.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2730.99,7006.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3610.45,6917.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4264.53,6985.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4932.63,6969.98");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5371.46,6976.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5748.26,6897.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=369.21,6719.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=726.04,6675.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1536.24,6619.06");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3037.17,6662.57");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4521.47,6689.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5944.29,6575.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=928.92,6303.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1875.71,6503.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2227.98,6277.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2843.99,6343.75");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3527.14,6477.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3876.53,6419.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5284.35,6409.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=40.37,6210.93");
    }

}

