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

network DADCA_ACK_n50_u16_a8000m2_9
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
        @display("i=misc/sensor;p=1122.08,7753.06");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1428.03,7994.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1706.93,7761.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2210.30,7867.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2887.79,7841.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3292.14,7765.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3672.99,7851.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4558.24,7848.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5164.26,7817.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=567.65,7667.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2535.25,7736.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4175.88,7597.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5914.83,7709.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1713.74,7407.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2377.50,7290.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2827.72,7419.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3350.06,7344.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3696.96,7493.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5050.69,7476.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5374.06,7308.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=40.24,7226.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=564.94,7070.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1037.82,7123.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1409.93,7114.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1989.87,7064.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3635.48,7016.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4306.08,7017.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4574.86,7245.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4952.86,7120.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5849.41,7038.43");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3126.00,6908.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5370.15,6783.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=90.02,6531.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=481.12,6701.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=950.70,6656.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1767.81,6755.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2377.80,6735.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2751.10,6729.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4109.89,6550.39");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4751.80,6648.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1557.08,6439.62");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1939.46,6403.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2324.21,6377.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2710.97,6365.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3217.19,6495.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3544.14,6312.86");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5841.11,6432.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=368.05,6218.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=686.78,6046.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1133.39,6247.58");
    }

}

