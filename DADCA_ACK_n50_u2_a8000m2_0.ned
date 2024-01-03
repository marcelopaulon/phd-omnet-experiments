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
        @display("i=misc/sensor;p=3481.05,7844.83");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4295.58,7809.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4853.22,7911.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5647.45,7941.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=401.65,7662.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=876.88,7603.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3956.13,7645.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4643.87,7601.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5393.72,7629.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1010.07,7267.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1587.55,7278.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1911.88,7459.02");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2513.55,7485.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3000.42,7397.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3427.27,7461.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4421.88,7267.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5107.94,7375.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=485.96,7076.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2099.51,7017.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2515.87,7065.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4945.04,7020.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5372.68,7108.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=85.59,6923.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1563.06,6892.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2850.17,6917.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3415.04,6986.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3977.05,6975.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4615.34,6865.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5928.55,6945.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1000.13,6673.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2180.95,6610.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2572.63,6645.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4397.39,6578.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4750.27,6535.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5289.59,6690.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=497.32,6372.97");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3209.58,6445.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3839.90,6451.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5641.94,6471.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1025.27,6246.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1664.25,6253.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2054.30,6161.61");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2564.42,6264.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2975.78,6056.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3540.51,6248.74");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4363.74,6153.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4949.57,6183.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5314.40,6259.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5823.64,6031.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=117.15,5861.10");
    }

}

