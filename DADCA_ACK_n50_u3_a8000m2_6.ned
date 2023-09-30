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

network DADCA_ACK_n50_u3_a8000m2_6
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
        @display("i=misc/sensor;p=971.54,7958.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1568.38,7901.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2456.57,7761.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4574.72,7856.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=686.98,7657.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2043.91,7614.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2824.43,7724.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3339.08,7720.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4929.77,7520.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5493.35,7658.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1375.16,7503.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1670.49,7277.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3900.93,7331.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4422.21,7264.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=256.66,7190.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=698.08,7217.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1053.97,7244.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2326.47,7213.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2969.24,7242.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3478.26,7056.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4789.62,7023.90");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5435.60,7223.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5796.76,7128.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1362.99,6953.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2704.65,6934.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=451.48,6541.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=980.60,6607.35");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1815.96,6722.50");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2350.32,6566.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3243.91,6634.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3972.93,6697.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4565.91,6616.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4965.23,6574.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5410.75,6732.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5863.43,6716.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1349.13,6433.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5326.09,6348.51");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1047.29,6214.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1555.88,6054.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1962.30,6140.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2518.09,6035.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2902.32,6103.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3671.73,6108.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4046.01,6210.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4518.05,6020.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5038.43,6051.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5663.90,6110.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=656.52,5947.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3130.76,5829.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=340.53,5745.12");
    }

}

