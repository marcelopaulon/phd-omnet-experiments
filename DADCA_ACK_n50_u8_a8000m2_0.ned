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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=997.06,7894.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1350.74,7859.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1721.44,7906.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2187.98,7881.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3450.32,7928.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5107.69,7890.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=643.84,7692.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2503.65,7724.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3030.35,7630.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5692.62,7681.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=158.56,7289.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=962.34,7354.21");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1341.03,7493.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1954.05,7421.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3369.93,7358.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5178.40,7427.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5635.67,7313.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1364.04,7046.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2372.08,7115.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2978.97,7219.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3950.22,7123.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=181.68,6815.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=654.94,6891.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1714.51,6897.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4530.64,6883.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5107.12,6946.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5641.58,6952.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1179.74,6548.93");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2287.43,6656.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2639.80,6667.30");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3355.10,6751.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4217.77,6560.44");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5412.72,6604.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=631.94,6419.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1733.18,6507.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3056.57,6429.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3647.78,6442.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4717.19,6483.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5906.43,6337.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=251.31,6053.18");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=893.05,6132.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1747.67,6122.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2286.68,6220.36");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2880.59,6041.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3358.81,6183.24");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4496.15,6050.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4988.74,6157.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1331.23,5866.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2407.73,5842.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3931.48,5935.72");
    }

}

