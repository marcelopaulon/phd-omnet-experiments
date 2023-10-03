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

network DADCA_ACK_n50_u16_a8000m2_3
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
        @display("i=misc/sensor;p=3998.00,7959.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4968.75,7753.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5621.30,7960.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=256.13,7691.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=974.65,7628.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1949.93,7734.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2338.29,7573.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2876.50,7561.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3387.80,7723.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4545.54,7692.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5664.03,7531.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1335.86,7361.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=527.42,7193.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1794.12,7063.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2155.99,7084.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3259.75,7013.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4198.95,7209.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5418.66,7115.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=171.77,6835.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=797.69,6965.12");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1391.29,6924.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2718.96,6848.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3984.70,6878.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4793.06,6959.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5163.43,6875.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5978.05,6869.44");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=564.19,6673.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1186.44,6535.48");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2114.88,6691.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3065.30,6590.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5502.68,6753.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=199.54,6424.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1666.61,6378.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2662.86,6502.67");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3502.87,6382.27");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4646.95,6413.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5856.97,6434.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=728.74,6063.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2461.98,6198.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3200.86,6066.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4126.10,6226.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4694.65,6026.40");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5111.21,6214.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=15.64,5943.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1206.77,5979.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2139.45,6002.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3941.06,5902.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4287.56,5832.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5486.60,6014.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1201.98,5541.21");
    }

}

