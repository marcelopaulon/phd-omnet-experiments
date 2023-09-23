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

network DADCA_ACK_n50_u4_a8000m2_3
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
        @display("i=misc/sensor;p=661.52,7903.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1225.66,7964.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1566.27,7754.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2056.06,7827.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2601.68,7978.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2977.28,7909.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3462.24,7802.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4643.73,7942.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5490.98,7983.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1185.68,7580.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4030.61,7664.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4988.39,7595.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=221.06,7394.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=814.88,7368.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1969.34,7258.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2437.00,7365.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2909.52,7437.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3585.33,7417.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4287.59,7360.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5518.46,7492.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=453.86,7111.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1234.52,7062.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1554.19,7230.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3186.55,7164.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3959.97,7177.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4903.16,7139.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5361.86,7019.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5736.16,7080.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1881.24,6797.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2901.93,6872.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3395.31,6802.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4461.77,6992.46");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=76.77,6642.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=883.49,6686.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1293.93,6532.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2707.51,6535.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3885.56,6677.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4331.81,6542.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4762.00,6744.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5223.72,6625.97");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1917.02,6352.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2300.23,6489.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3362.87,6436.39");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4663.42,6397.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=614.75,6242.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3671.78,6228.92");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4208.19,6200.19");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4638.63,6040.16");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4974.92,6169.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5666.33,6147.16");
    }

}

